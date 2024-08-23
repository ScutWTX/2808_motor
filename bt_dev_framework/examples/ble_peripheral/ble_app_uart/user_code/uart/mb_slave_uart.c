#include "prjconfig.h"

#include "app_uart.h"
#include "app_timer.h"

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

/** \brief 从机状态 */
enum mb_slaver_state {
    MB_SLAVER_STATE_IDLE,      /**< \brief 空闲状态 */
    MB_SLAVER_STATE_SENDING,   /**< \brief 发送状态 */
    MB_SLAVER_STATE_RECEIVING  /**< \brief 接收状态 */
};


/** \brief t35定时器 */
APP_TIMER_DEF(mb_uart_t35_timer);

static void __mb_send(uint8_t *tx_buf, uint16_t tx_len);
void mb_slave_uart_rxdata_handling(uint8_t const *p_data, uint16_t len);


struct svo_uart_slave {
    enum mb_slaver_state state;
    struct ringbuf m_uart_ringbuf;
    
    uint8_t rxdata_buf[PRJCONF_MODBUS_SLAVE_BLE_CACHE_SIZE];
    
    uint8_t  rx_buf[PRJCONF_UART_RX_BUF_SIZE]; /**< \brief 接收缓冲区   */
    uint16_t rx_len;                           /**< \brief 接收数据长度 */
	
	uint8_t tx_tmp_buf[128];
	uint8_t rx_tmp_buf[128];
    
    uint32_t rx_last_tick;                     /**< \brief 上一次接收到数据时的时间点 */
    uint32_t rx_cur_tick;                      /**< \brief 当前接收到数据时的时间点   */
    bool     rx_timeout_check;                 /**< \brief 接收超时检查               */
    
    uint8_t  event_type_t35_timeout_poll;      /**< \brief T35超时轮询事件 */
    uint8_t  event_type_rxdata_handling;       /**< \brie 接收数据处理 */
}; 

static struct svo_uart_slave m_uart_slave;

/** \brief import from main.c */
extern event_loop_t el;

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
    switch (addr) {
        case MB_SLAVE_USER_DIR:
            wfr_set_dir(coil_val ? WFR_DIR_PULL : WFR_DIR_PUSH);
            break;
        
        case MB_SLAVE_USER_STARTUP:
            wfr_startup(coil_val ? true : false);
            break;
        
        case MB_SLAVE_USER_STOP_ROLLBACK:
            wfr_set_rollback_when_stop(coil_val ? true : false);
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
        __mb_error_code_respond(MB_EXC_ILLEGAL_DATA_VALUE, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
		
		return MB_EXC_ILLEGAL_DATA_VALUE;
	}
	
    u16crc = mb_crc16(p_buf, len - 2);	
	
	if ((LOW_BYTE(u16crc) != p_buf[len - 2] || HIGH_BYTE(u16crc) != p_buf[len - 1])) {
        __mb_error_code_respond(MB_EXC_ILLEGAL_DATA_VALUE, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
		
		return MB_EXC_ILLEGAL_DATA_VALUE;
	}
	
	u16addr = WORD(p_buf[2], p_buf[3]);
	
	switch (p_buf[1]) {
		case MB_FC_READ_HOLDING_REGS:
			u16qty = WORD(p_buf[4], p_buf[5]);

			if ((u16addr + u16qty - 1) > MB_SLAVE_USER_REG_MAX_ADDR || u16addr < MB_SLAVE_USER_REG_MIN_ADDR || u16qty < 1) {
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				m_uart_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_uart_slave.tx_tmp_buf[1] = MB_FC_READ_HOLDING_REGS;
				m_uart_slave.tx_tmp_buf[2] = u16qty * 2;

				for (i = 0, j = 0; i < u16qty; j = j + 2, i++) {
					u16value = __mb_slave_get_reg(u16addr + i);
					m_uart_slave.tx_tmp_buf[3 + j] = HIGH_BYTE(u16value);
					m_uart_slave.tx_tmp_buf[4 + j] = LOW_BYTE(u16value);
				}

				u16txbuff_length = 3 + m_uart_slave.tx_tmp_buf[2];

				u16crc = mb_crc16(m_uart_slave.tx_tmp_buf, u16txbuff_length);

				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
			}

			break;
			
		case MB_FC_WRITE_COILS:
			u16qty = WORD(p_buf[4], p_buf[5]);
		
			if ((u16addr + u16qty -1) > MB_SLAVE_USER_COIL_MAX_ADDR || u16qty < 1) {
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				m_uart_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_uart_slave.tx_tmp_buf[1] = MB_FC_WRITE_COILS;
				m_uart_slave.tx_tmp_buf[2] = p_buf[2];
				m_uart_slave.tx_tmp_buf[3] = p_buf[3];
				m_uart_slave.tx_tmp_buf[4] = p_buf[4];
				m_uart_slave.tx_tmp_buf[5] = p_buf[5];
				
				u16txbuff_length = 6;
				
                u16crc = mb_crc16(m_uart_slave.tx_tmp_buf, u16txbuff_length);

				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
                
                __mb_slave_update_coils(u16addr, u16qty, &p_buf[7]);
			}

			break;

		case MB_FC_WRITE_REGS:
			u16qty = WORD(p_buf[4], p_buf[5]);
		
			if ((u16addr + u16qty-1) > MB_SLAVE_USER_REG_MAX_ADDR || u16addr < MB_SLAVE_USER_REG_MIN_ADDR || u16qty < 1) {
				__mb_error_code_respond(MB_EXC_ILLEGAL_DATA_ADDRESS, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
				
                ret = MB_EXC_ILLEGAL_DATA_ADDRESS;
			} else {
				for (i = 0, j = 0; i < p_buf[6] / 2; i++, j += 2) {
					u16value = WORD(p_buf[7 + j], p_buf[8 + j]);
					__mb_slave_update_reg(u16addr + i, u16value);
				}

				m_uart_slave.tx_tmp_buf[0] = MB_SLAVE_USER_ADDR;
				m_uart_slave.tx_tmp_buf[1] = MB_FC_WRITE_REGS;
				m_uart_slave.tx_tmp_buf[2] = p_buf[2];
				m_uart_slave.tx_tmp_buf[3] = p_buf[3];
				m_uart_slave.tx_tmp_buf[4] = p_buf[4];
				m_uart_slave.tx_tmp_buf[5] = p_buf[5];
				
				u16txbuff_length = 6;		
				
                u16crc = mb_crc16(m_uart_slave.tx_tmp_buf, u16txbuff_length);
				
				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = LOW_BYTE(u16crc);
				m_uart_slave.tx_tmp_buf[u16txbuff_length++] = HIGH_BYTE(u16crc);
			}

			break;

		default: {
			__mb_error_code_respond(MB_EXC_ILLEGAL_FUNCTION, p_buf, m_uart_slave.tx_tmp_buf, &u16txbuff_length);
			ret = MB_EXC_ILLEGAL_FUNCTION;
        }
        break;
	}
    
    __mb_send(m_uart_slave.tx_tmp_buf, u16txbuff_length);
	
	return ret;
}


/** \brief 串口数据处理 */
static void mb_uart_rxdata_handler(uint8_t *p_data, uint16_t length)
{
    uint32_t len = 0;
    
	ringbuf_get_with_length(&m_uart_slave.m_uart_ringbuf, m_uart_slave.rx_tmp_buf, &len);
	
	__mb_data_handler(m_uart_slave.rx_tmp_buf, len);
}

/** \brief 开启modbus T35超时检查 */
static void __mb_uart_t35_enable(void)
{
    m_uart_slave.rx_timeout_check = true;
    m_uart_slave.rx_last_tick = app_timer_cnt_get();
}

/** \brief 停止modbus T35超时检查 */
static void __mb_uart_t35_disable(void)
{
    m_uart_slave.rx_timeout_check = false;
}

void mb_uart_t35_timeout_poll(uint8_t *p_data, uint16_t length)
{
    if (m_uart_slave.rx_timeout_check) {
        m_uart_slave.rx_cur_tick = app_timer_cnt_get();
        if (app_timer_cnt_diff_compute(m_uart_slave.rx_cur_tick, m_uart_slave.rx_last_tick) >= APP_TIMER_TICKS(PRJCONF_MODBUS_T35)) {
            __mb_uart_t35_disable();
            
            app_timer_stop(mb_uart_t35_timer);
            
            mb_slave_uart_rxdata_handling(m_uart_slave.rx_buf, m_uart_slave.rx_len);
            
            m_uart_slave.rx_len = 0;
        }
    }
}

/** \brief modbus T35超时轮询 */
static void __mb_uart_t35_timeout_poll(void *p_context)
{
    pending_event_t ble_recv_data;
    
    ble_recv_data.event_type = m_uart_slave.event_type_t35_timeout_poll;
    
    pending_event_report(&ble_recv_data);
}

/** \brief modbus发送 */
static void __mb_send(uint8_t *tx_buf, uint16_t tx_len)
{
    uint16_t i;
    
    m_uart_slave.state = MB_SLAVER_STATE_SENDING;
	
    app_uart_flush();
    
    PRJCONF_UART_TX_ENABLE;
    
    for (i = 0; i < tx_len; ++i) 
        while (app_uart_put(tx_buf[i]) != NRF_SUCCESS);
}

/**
 * \brief Function for handling app_uart events
 */
static void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t c;
    
    switch (p_event->evt_type) {
        case APP_UART_DATA_READY:
            if (m_uart_slave.state == MB_SLAVER_STATE_IDLE) {
                app_uart_get(&m_uart_slave.rx_buf[m_uart_slave.rx_len++]);
                app_timer_start(mb_uart_t35_timer, APP_TIMER_TICKS(PRJCONF_MODBUS_T35), NULL);
                __mb_uart_t35_enable();
            } else {
                app_uart_get(&m_uart_slave.rx_buf[m_uart_slave.rx_len++]);
                __mb_uart_t35_enable();
            }
            
            break;

		case APP_UART_TX_EMPTY:
            PRJCONF_UART_RX_ENABLE;
        
            app_uart_rx_enable();
        
            m_uart_slave.state = MB_SLAVER_STATE_IDLE;
			break;
            
        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
           break;

        default:
            break;
    }
}

 
void mb_slave_uart_init(void)
{
    uint32_t err_code;
    app_uart_comm_params_t const comm_params =
    {
		.rx_pin_no	  = PRJCONF_UART_RX_PIN,
		.tx_pin_no	  = PRJCONF_UART_TX_PIN,

        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
         
        .use_parity   = PRJCONF_UART_PARITY,


#if defined (UART_PRESENT)
        .baud_rate    = PRJCONF_UART_BAUDRATE
#else
        .baud_rate    = NRF_UART_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       PRJCONF_UART_RX_BUF_SIZE,
                       PRJCONF_UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    
    APP_ERROR_CHECK(err_code);
	
	/* 设置485使能引脚 */
	nrf_gpio_cfg_output(PRJCONF_UART_DE);
    
    ringbuf_init(&m_uart_slave.m_uart_ringbuf, m_uart_slave.rxdata_buf, PRJCONF_MODBUS_SLAVE_UART_CACHE_SIZE);
     
    m_uart_slave.state = MB_SLAVER_STATE_IDLE;
    
    m_uart_slave.event_type_t35_timeout_poll = pending_event_type_alloc();
    if (!PENDING_EVENT_TYPE_ALLOC_CHECK(m_uart_slave.event_type_t35_timeout_poll)) {
        NRF_LOG_INFO("modbus t35 timeout poll event type alloc failed.");
    } else {
        pending_event_handler_register(m_uart_slave.event_type_t35_timeout_poll, mb_uart_t35_timeout_poll);
    }
    
    m_uart_slave.event_type_rxdata_handling = pending_event_type_alloc();
    if (!PENDING_EVENT_TYPE_ALLOC_CHECK(m_uart_slave.event_type_rxdata_handling)) {
        NRF_LOG_INFO("modbus t35 timeout poll event type alloc failed.");
    } else {
        pending_event_handler_register(m_uart_slave.event_type_rxdata_handling, mb_uart_rxdata_handler);
    }
    
    err_code = app_timer_create(&mb_uart_t35_timer, APP_TIMER_MODE_REPEATED, __mb_uart_t35_timeout_poll);
	APP_ERROR_CHECK(err_code);
}


void mb_slave_uart_rxdata_handling(uint8_t const *p_data, uint16_t len)
{
    pending_event_t ble_recv_data;
    
    ringbuf_put_with_length(&m_uart_slave.m_uart_ringbuf, p_data, len);

    ble_recv_data.event_type = m_uart_slave.event_type_rxdata_handling;
    
    pending_event_report(&ble_recv_data);
}
