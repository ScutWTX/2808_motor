#include "prjconfig.h"

#include "mb_crc.h"
#include "mb_common.h"
#include "mb_slave_user_def.h"

#include "ringbuf.h"
#include "event.h"
#include "event_pool.h"

#include "wirefeeder.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

struct svo_ble_slave {
    struct ringbuf m_ble_ringbuf;
    uint8_t ble_rxdata_buf[PRJCONF_MODBUS_SLAVE_BLE_CACHE_SIZE];
	
	uint8_t tx_tmp_buf[128];
	uint8_t rx_tmp_buf[128];
    
    event_t tmp_event;
    bool tmp_event_used;
};

static struct svo_ble_slave m_ble_slave;

/** \brief import from main.c */
extern event_loop_t el;

/** \brief import from main.c */
extern void ble_send_string(uint8_t *p_buf, uint16_t len);

/** \brief 获取寄存器值 */
static uint16_t __mb_slave_get_reg(uint16_t addr)
{
	uint16_t ret = 0;

    switch (addr) {
        case MB_SLAVE_USER_STATUS:
            ret = (uint16_t)wfr_get_status();
            break;
        default:
            break;
    }
	
	return ret;
}

/** \brief 处理线圈 */
static void __mb_slave_update_coil(uint16_t addr, uint8_t coil_val)
{
    NRF_LOG_INFO("addr:%x", addr);
    switch (addr) {
        case MB_SLAVE_USER_DIR:
            wfr_set_dir(coil_val ? WFR_DIR_PULL : WFR_DIR_PUSH);
            break;
        
        case MB_SLAVE_USER_STARTUP:
            wfr_startup(coil_val ? true : false);
            break;
        
        case MB_SLAVE_USER_STOP_ROLLBACK:
            wfr_set_rollback_when_stop(coil_val ? true : false);
//            seg_ctrl();
            break;
        
        case MB_SLAVE_USER_SEG_SWITCH:
            seg_switch();
            break;
        case MB_SLAVE_USER_SEG_STARTUP:
            if (coil_val) {
                NRF_LOG_INFO("seg_start...");
            } else {
                NRF_LOG_INFO("seg_stop...");
            }
            seg_ctrl(coil_val ? true : false);
            break;
        
        default:
            break;
    }
}

static void __mb_slave_update_coils(uint16_t addr, uint16_t cnt_of_coils, uint8_t *p_buf)
{
	int i;
	
    for (i = 0; i < cnt_of_coils; ++i)
        __mb_slave_update_coil(addr + i, (p_buf[i / 8] & (1 << (i % 8))) ? 1 : 0);
}

static void __mb_slave_update_reg(uint16_t addr, uint16_t val)
{
    static uint16_t __max_id = 0, __cur_id = 0;
    
//    NRF_LOG_INFO("addr:%x, value:%d", addr, val);
    
    switch (addr) {
        case MB_SLAVE_USER_SPEED:
            wfr_set_push_speed(val);
            break;
        case MB_SLAVE_USER_PULL_SPEED:
            wfr_set_pull_speed(val);
            break;
        case MB_SLAVE_USER_PULL_TIME:
            wfr_set_pull_time(val);
            break;
        
        case MB_SLAVE_USER_SEG_ID:
            if (val >= __max_id)
                __max_id = val;
            
            __cur_id = val;
            
            NRF_LOG_INFO("id:%d", __cur_id);
            break;
        
//        case MB_SLAVE_USER_SEG_DELTA:
//        case MB_SLAVE_USER_SEG_SLOPE_TIME:
//        case MB_SLAVE_USER_SEG_DURATION:
                 
        case MB_SLAVE_USER_ISPERIOD:   
        case MB_SLAVE_USER_FEEDINGSPEED:
        case MB_SLAVE_USER_DRAWINGSPEED: 
        case MB_SLAVE_USER_FEEDINGTIME:  
        case MB_SLAVE_USER_DRAWINGTIME:  
            NRF_LOG_INFO("data:%d", val);
            set_seg_data(__cur_id, (addr - MB_SLAVE_USER_SEG_ID), val);
            if (__cur_id == 1 && addr == MB_SLAVE_USER_DRAWINGTIME) {
                set_seg_max_count(__max_id);
                NRF_LOG_INFO("max seg count:%d", __max_id);
                __max_id = 0;
                __cur_id = 0;
            }
            break;
        
        default:
            break;
    }
}


/** \brief 异常响应打包 */
static void __mb_error_code_respond(uint8_t errno, uint8_t *rx_buf, uint8_t *respond_buf, uint16_t *respond_buf_len)
{
    respond_buf[0]   = rx_buf[0];
    respond_buf[1]   = rx_buf[1] | 0x80;
    respond_buf[2]   = errno;
	
    *respond_buf_len = 3;	
	
    uint16_t u16crc = mb_crc16(respond_buf, (*respond_buf_len));
	
    respond_buf[(*respond_buf_len)++] = LOW_BYTE(u16crc);
    respond_buf[(*respond_buf_len)++] = HIGH_BYTE(u16crc); 
}

/** \brief modbus从机数据处理 */
static mb_exception_t __mb_data_handler(uint8_t *p_buf, uint16_t len)
{
	uint16_t i, j;
	uint16_t u16addr, u16qty, u16txbuff_length, u16value;
	uint16_t u16crc;
	
	mb_exception_t ret = MB_EXC_NONE;

	if (p_buf == NULL || len == 0)
		return MB_EXC_INVAL_PARAM;

	if (p_buf[0] != MB_SLAVE_USER_ADDR) {
        __mb_error_code_respond(MB_EXC_ILLEGAL_DATA_VALUE, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
		
		return MB_EXC_ILLEGAL_DATA_VALUE;
	}
	
    u16crc = mb_crc16(p_buf, len - 2);	
	
	if ((LOW_BYTE(u16crc) != p_buf[len - 2] || HIGH_BYTE(u16crc) != p_buf[len - 1])) {
        __mb_error_code_respond(MB_EXC_ILLEGAL_DATA_VALUE, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
		
		return MB_EXC_ILLEGAL_DATA_VALUE;
	}
	
	u16addr = WORD(p_buf[2], p_buf[3]);
	
	switch (p_buf[1]) {
		case MB_FC_READ_HOLDING_REGS:
			u16qty = WORD(p_buf[4], p_buf[5]);

			if ((u16addr + u16qty - 1) > MB_SLAVE_USER_REG_MAX_ADDR || u16addr < MB_SLAVE_USER_REG_MIN_ADDR || u16qty < 1) {
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				m_ble_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_ble_slave.tx_tmp_buf[1] = MB_FC_READ_HOLDING_REGS;
				m_ble_slave.tx_tmp_buf[2] = u16qty * 2;

				for (i = 0, j = 0; i < u16qty; j = j + 2, i++) {
					u16value = __mb_slave_get_reg(u16addr + i);
					m_ble_slave.tx_tmp_buf[3 + j] = HIGH_BYTE(u16value);
					m_ble_slave.tx_tmp_buf[4 + j] = LOW_BYTE(u16value);
				}

				u16txbuff_length = 3 + m_ble_slave.tx_tmp_buf[2];

				u16crc = mb_crc16(m_ble_slave.tx_tmp_buf, u16txbuff_length);

				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
			}

			break;
			
		case MB_FC_WRITE_COILS:
			u16qty = WORD(p_buf[4], p_buf[5]);
		
			if ((u16addr + u16qty -1) > MB_SLAVE_USER_COIL_MAX_ADDR || u16qty < 1) {
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				m_ble_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_ble_slave.tx_tmp_buf[1] = MB_FC_WRITE_COILS;
				m_ble_slave.tx_tmp_buf[2] = p_buf[2];
				m_ble_slave.tx_tmp_buf[3] = p_buf[3];
				m_ble_slave.tx_tmp_buf[4] = p_buf[4];
				m_ble_slave.tx_tmp_buf[5] = p_buf[5];
				
				u16txbuff_length = 6;
				
                u16crc = mb_crc16(m_ble_slave.tx_tmp_buf, u16txbuff_length);

				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
                
                __mb_slave_update_coils(u16addr, u16qty, &p_buf[7]);
			}

			break;

		case MB_FC_WRITE_REGS:
			u16qty = WORD(p_buf[4], p_buf[5]);
		
			if ((u16addr + u16qty-1) > MB_SLAVE_USER_REG_MAX_ADDR || u16addr < MB_SLAVE_USER_REG_MIN_ADDR || u16qty < 1) {                
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				for (i = 0, j = 0; i < p_buf[6] / 2; i++, j += 2) {
					u16value = WORD(p_buf[7 + j], p_buf[8 + j]);
					__mb_slave_update_reg(u16addr + i, u16value);
				}

				m_ble_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_ble_slave.tx_tmp_buf[1] = MB_FC_WRITE_REGS;
				m_ble_slave.tx_tmp_buf[2] = p_buf[2];
				m_ble_slave.tx_tmp_buf[3] = p_buf[3];
				m_ble_slave.tx_tmp_buf[4] = p_buf[4];
				m_ble_slave.tx_tmp_buf[5] = p_buf[5];
				
				u16txbuff_length = 6;		
				
                u16crc = mb_crc16(m_ble_slave.tx_tmp_buf, u16txbuff_length);
				
				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_ble_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
			}

			break;

		default: {
			__mb_error_code_respond(MB_EXC_ILLEGAL_FUNCTION, p_buf, m_ble_slave.tx_tmp_buf, &u16txbuff_length);
			ret = MB_EXC_ILLEGAL_FUNCTION;
        }
        break;
	}

	ble_send_string(m_ble_slave.tx_tmp_buf, u16txbuff_length);
	
	return ret;
}


/** \brief 蓝牙数据处理 */
static void __ble_rxdata_handler(void *p_context, event_t *e)
{
    uint32_t len = 0;
    
	ringbuf_get_with_length(&m_ble_slave.m_ble_ringbuf, m_ble_slave.rx_tmp_buf, &len);
	
	__mb_data_handler(m_ble_slave.rx_tmp_buf, len);
    
	if (m_ble_slave.tmp_event_used)
        m_ble_slave.tmp_event_used = false;
    else
        event_pool_free(e);
}

 
void mb_slave_ble_init(void)
{
    ringbuf_init(&m_ble_slave.m_ble_ringbuf, m_ble_slave.ble_rxdata_buf, PRJCONF_MODBUS_SLAVE_BLE_CACHE_SIZE);
    
    m_ble_slave.tmp_event_used = false;
}


void mb_slave_ble_rxdata_handling(uint8_t const *p_data, uint16_t len)
{
    event_t *e;
    
    ringbuf_put_with_length(&m_ble_slave.m_ble_ringbuf, p_data, len);
    
    e = event_pool_alloc();
    if (!e) {
        e = &m_ble_slave.tmp_event;
        m_ble_slave.tmp_event_used = true;
    }
    
    event_init(e, __ble_rxdata_handler, NULL, EVT_MAKE_PRIORITY(EVT_HIGHEST_GRP_PRIO, 0));
    event_loop_event_post(&el, e);
}
