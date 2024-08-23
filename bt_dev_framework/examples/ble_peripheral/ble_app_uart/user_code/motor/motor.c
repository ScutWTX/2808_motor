/**
 * \file
 * \brief motor control
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-12  lwx, first implementation
 * \endinternal
 */
 
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h" 
 
#include "app_timer.h"

#include "motor.h"
#include "mb_common.h"
#include "mb_master_uart.h"
#include "mb_master.h"


/** \brief ���״̬ */
enum motor_stat {
    MOTOR_STAT_INIT = 0,                /**< \brief �����ʼ�� */
    MOTOR_STAT_CONFIG_MOTOR_EEPROM,     /**< \brief ����EEPROM��� */
    MOTOR_STAT_CHECK_MOTOR_IS_READY,    /**< \brief ������Ƿ������� */
    MOTOR_STAT_ENABLE_IO,               /**< \brief ʹ��IO��� */
    MOTOR_STAT_READY                    /**< \brief ������� */
};

/** \brief ������� */
enum motor_action {
    MOTOR_ACTION_NONE = 0,              /**< \brief �޶��� */
    MOTOR_ACTION_CONFIG_MOTOR_EEPROM,   /**< \brief ����EEPROM */
    MOTOR_ACTION_CHECK_MOTOR_IS_READY,  /**< \brief ������Ƿ���� */
    MOTOR_ACTION_ENABLE_IO,             /**< \brief ʹ��IO */
    MOTOR_ACTION_ENABLE,                /**< \brief ʹ�ܵ�� */
	MOTOR_ACTION_GET_POS,               /**< \brief ��ȡλ�� */
	MOTOR_ACTION_GET_ALARM_CODE,        /**< \brief ��ȡ����״̬ */
	MOTOR_ACTION_SET_SPEED,             /**< \brief �����ٶ� */
};

/** \brief �ŷ�״̬ */
enum servo_stat {
    SVO_STAT_IDLE = 0,
    SVO_STAT_WORKING
};

/** \brief �����һ��״̬ */
#define __MOTOR_NEXT_STAT(p_dev) p_dev->stat += 1

/** \brief �����һ��״̬ */
#define __MOTOR_LAST_STAT(p_dev) p_dev->stat -= 1

/** \brief �������״̬*/
#define __MOTOR_SET_STAT(p_dev, stat) p_dev->stat = stat

/** \brief �ŷ���һ��״̬ */
#define __SVO_NEXT_STAT(p_svo, stat) p_svo->stat = stat


#define __SVO_EXCEPTION_ROUGH_FB (1 << 0) /**< \brief �ֵ�ǰ����λ�쳣 */
#define __SVO_EXCEPTION_ROUGH_UD (1 << 1) /**< \brief �ֵ�������λ�쳣 */
#define __SVO_EXCEPTION_FINE_LR  (1 << 2) /**< \brief ����������λ�쳣 */
#define __SVO_EXCEPTION_FINE_UD  (1 << 3) /**< \brief ����������λ�쳣 */


/** \brief ����豸 */
struct motor_dev {
    enum motor_stat stat;       /**< \brief ���״̬               */
    enum motor_axis_idx idx;    /**< \brief ������                 */
    enum motor_dir_pol luf_pol; /**< \brief (���ϡ�ǰ)������   */
    
	uint16_t    max_speed;      /**< \brief ����ٶ�,��λ:(mm/min) */
    uint16_t   auto_speed;      /**< \brief �Զ��ٶ�,��λ:(mm/min) */
    uint16_t manual_speed;      /**< \brief �ֶ��ٶ�,��λ:(mm/min) */
	
	uint16_t alarm_code;        /**< \brief ��������               */
                                                                 
    int32_t position;           /**< \brief λ����Ϣ               */
                                
    uint8_t addr;               /**< \brief ���modbus��ַ         */
};

/** \brief �ڶ������� */
struct wiggler_params {
    uint16_t left_range;               /**< \brief �ڶ�����ڷ�    */
    uint16_t left_speed;               /**< \brief �ڶ�������ٶ�  */
    uint16_t left_stop_time;           /**< \brief �ڶ�����ͣʱ��  */
    uint16_t right_range;              /**< \brief �ڶ����Ұڷ�    */
    uint16_t right_speed;              /**< \brief �ڶ����Ұ��ٶ�  */
    uint16_t right_stop_time;          /**< \brief �ڶ�����ͣʱ��  */
    uint16_t mid_stop_time;            /**< \brief �ڶ�����ͣʱ��  */
    uint16_t start_delay;              /**< \brief �ڶ���������ʱ  */
    uint16_t stop_delay;               /**< \brief �ڶ���ֹͣ��ʱ  */
};

/** \brief �ŷ��豸 */
struct servo {
    struct motor_dev motors[MOTOR_MAX_COUNT]; /**< \brief ����豸         */
    
    uint16_t start_delay;                     /**< \brief ���˶���������ʱ */
    uint16_t stop_delay;                      /**< \brief ���˶���ֹͣ��ʱ */
     
    struct wiggler_params wiggler;            /**< \brief �ڶ���           */

    uint16_t exception;                       /**< \brief �쳣��           */
    enum servo_stat stat;                     /**< \brief ״̬             */
	
	uint32_t last_tick;                       /**< \brief ��һ��ʱ���     */
	uint32_t cur_tick;                        /**< \brief ��ǰʱ���       */
};

/** �ŷ������� */
static struct servo m_servo = {
    {
        { MOTOR_STAT_INIT, MOTOR_AXIS_X1, PRJCONF_MOTOR_X1_LUF_POL, PRJCONF_MOTOR_X1_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_X1_MODBUS_ADDR },
        { MOTOR_STAT_INIT, MOTOR_AXIS_Y1, PRJCONF_MOTOR_Y1_LUF_POL, PRJCONF_MOTOR_Y1_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_Y1_MODBUS_ADDR },
        { MOTOR_STAT_INIT, MOTOR_AXIS_Z1, PRJCONF_MOTOR_Z1_LUF_POL, PRJCONF_MOTOR_Z1_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_Z1_MODBUS_ADDR },
        { MOTOR_STAT_INIT, MOTOR_AXIS_X2, PRJCONF_MOTOR_X2_LUF_POL, PRJCONF_MOTOR_X2_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_X2_MODBUS_ADDR },
        { MOTOR_STAT_INIT, MOTOR_AXIS_Y2, PRJCONF_MOTOR_Y2_LUF_POL, PRJCONF_MOTOR_Y2_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_Y2_MODBUS_ADDR },
        { MOTOR_STAT_INIT, MOTOR_AXIS_Z2, PRJCONF_MOTOR_Z2_LUF_POL, PRJCONF_MOTOR_Z2_MAX_SPEED, 0, 0, 0, 0, PRJCONF_MOTOR_Z2_MODBUS_ADDR },
    }, 
    
    0, 0,
    
    { 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    
    0
};

/** \brief �����ȡ��ʱ�� */
APP_TIMER_DEF(motor_reading_status_timer);

/************************ ������� *******************************************/

#define __MOTOR_AUX_FUNC_REG_ADDR 0x023C /**< \brief �������ܼĴ��� */
#define __MOTOR_AUX_FUNC_VAL      0x0005 /**< \brief ����������粻������eeprom */

/** \brief ���õ�������ϵ�󲻱���(doc: P316) */
static void __motor_config_not_store_in_eeprom(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_AUX_FUNC_VAL;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_AUX_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len);    
}

#define __MOTOR_DO_STAT_REG_ADDR 0x005C /**< \brief �������������(DO)�ź�״̬��ʾ */
#define __MOTOR_DO_STAT_READY    0x0001 /**< \brief ����������ʱbit0��1 */

/** \brief ��ȡDO�ź�״̬(doc: P270) */
static void __motor_get_do_status(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_DO_STAT_REG_ADDR, NULL, 1, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_DI_SOURCE_REG_ADDR 0x030C /**< \brief ����ӵ�(DI)��Դ���ƿ��� */
#define __MOTOR_DI_SOURCE_VAL      0x0007 /**< \brief ����ֵ */

/** \brief ����DI��Դ��ʹ��bit0~bit2����(DI1~DI3����Ӧ0x040E��ַ��bit0~bit2)(doc:P338) */
static void __motor_config_di_source(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_SOURCE_VAL;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_SOURCE_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

#define __MOTOR_DI_MULTI_FUNC_REG_ADDR 0x040E /**< \brief ���ֽ������ع��� */
#define __MOTOR_DI_MULTI_FUNC_IDLE     0x0001 /**< \brief ʹ��(ֹͣ) */
#define __MOTOR_DI_MULTI_FUNC_CW       0x0007 /**< \brief ��ת */
#define __MOTOR_DI_MULTI_FUNC_CCW      0x0003 /**< \brief ��ת */

/** \brief ʹ�ܵ��(ֹͣ���)(doc:P345) */
static void __motor_stop(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_IDLE;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/** \brief �����ת */
static void __motor_cw(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_CW;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/** \brief �����ת */
static void __motor_ccw(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_CCW;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

#define __MOTOR_STAT_MONITOR_CACHE_POS_REG_ADDR 0x0012 /**< \brief ״̬��ػ�����(λ����Ϣ) 32bits */

/** \brief ��ȡ���λ����Ϣ(doc:P262) */
static void __motor_get_pos(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_STAT_MONITOR_CACHE_POS_REG_ADDR, NULL, 2, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_ALARM_CODE_REG_ADDR 0x0002 /**< \brief �������������� */
#define __MOTOR_ALARM_CODE_AL014    0x14   /**< \brief �������쳣 */
#define __MOTOR_ALARM_CODE_AL015    0x15   /**< \brief �������쳣 */

/** \brief ��ȡ�����������(doc:P260,P570) */
static void __motor_get_alarm_code(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_ALARM_CODE_REG_ADDR, NULL, 1, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_JOG_SPEED_REG_ADDR 0x040A /**< \brief JOG(�綯)����,�ٶȼĴ�����ַ,��λ:rpm */

/** \brief �����ٶ�ֵ */
static void __motor_set_jog_speed(uint8_t sid, uint16_t speed)
{
    uint8_t __pdu[8];
    uint16_t len;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_JOG_SPEED_REG_ADDR, &speed, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/************************ �ڶ������� *****************************************/

#define __WIGGLER_LEFT_RANGE_REG_ADDR  0x0636 /**< \brief �ڶ�����ڷ��Ĵ�����ַ   */
#define __WIGGLER_LEFT_SPEED_REG_ADDR  0x0590 /**< \brief �ڶ�������ٶȼĴ�����ַ */
#define __WIGGLER_LEFT_DELAY_REG_ADDR  0x056C /**< \brief �ڶ�����ͣʱ��Ĵ�����ַ */
#define __WIGGLER_RIGHT_RANGE_REG_ADDR 0x062E /**< \brief �ڶ����Ұڷ��Ĵ�����ַ   */
#define __WIGGLER_RIGHT_SPEED_REG_ADDR 0x058E /**< \brief �ڶ����Ұ��ٶȼĴ�����ַ */
#define __WIGGLER_RIGHT_DELAY_REG_ADDR 0x056E /**< \brief �ڶ�����ͣʱ��Ĵ�����ַ */


/** \brief ͨ������ӻ���ַ�ҵ���Ӧ�ĵ���豸 */
struct motor_dev *__motor_get(uint8_t sid)
{
    int i;
    
    for (i = 0; i < MOTOR_MAX_COUNT; ++i) {
        if (sid == m_servo.motors[i].addr)
            return &m_servo.motors[i];
    }
    
    return NULL;
}

/** \brief ���ݹ�����Ͳ�����ַ�жϸô�����Ϊ�ĸ����� */
static enum motor_action __motor_get_action(uint8_t fc, uint16_t addr)
{
    if (MB_FC_WRITE_REG == fc && __MOTOR_AUX_FUNC_REG_ADDR == addr)
        return MOTOR_ACTION_CONFIG_MOTOR_EEPROM;
    
    if (MB_FC_READ_HOLDING_REGS == fc && __MOTOR_DO_STAT_REG_ADDR == addr) 
        return MOTOR_ACTION_CHECK_MOTOR_IS_READY;
    
    if (MB_FC_WRITE_REG == fc && __MOTOR_DI_SOURCE_REG_ADDR == addr)
        return MOTOR_ACTION_ENABLE_IO;
    
    if (MB_FC_WRITE_REG == fc && __MOTOR_DI_MULTI_FUNC_REG_ADDR == addr)
        return MOTOR_ACTION_ENABLE;
	
	if (MB_FC_READ_HOLDING_REGS == fc && __MOTOR_STAT_MONITOR_CACHE_POS_REG_ADDR == addr)
		return MOTOR_ACTION_GET_POS;
	
	if (MB_FC_READ_HOLDING_REGS == fc && __MOTOR_ALARM_CODE_REG_ADDR == addr)
		return MOTOR_ACTION_GET_ALARM_CODE;
    
    return MOTOR_ACTION_NONE;
}

/** \brief ���״̬�� */
static void __motor_fsm(struct motor_dev *p_dev, enum motor_action action, mb_exception_t action_ret, uint16_t *p_data, uint16_t len)
{
    if (NULL == p_dev)
        return;

    //�����������������ݲ������Ƿ���ת����һ��״̬
	if (MOTOR_ACTION_NONE != action) {
        switch(action) {
            case MOTOR_ACTION_CHECK_MOTOR_IS_READY:
                if (MB_EXC_NONE == action_ret && ((p_data[0] & __MOTOR_DO_STAT_READY) == __MOTOR_DO_STAT_READY)) {
                    __MOTOR_NEXT_STAT(p_dev);
                }
                break;
        
            case MOTOR_ACTION_CONFIG_MOTOR_EEPROM:
            case MOTOR_ACTION_ENABLE_IO:
            case MOTOR_ACTION_ENABLE:
                if (MB_EXC_NONE == action_ret)
                    __MOTOR_NEXT_STAT(p_dev);
                break;
				
	        case MOTOR_ACTION_GET_POS:
				p_dev->position = (int32_t)(p_data[0] | (p_data[1] << 16));
				break;
			
			case MOTOR_ACTION_GET_ALARM_CODE:
				p_dev->alarm_code = p_data[0];
            
                switch (p_dev->idx) {
                    case PRJCONF_ROUGH_FB:
                        if (p_dev->alarm_code == __MOTOR_ALARM_CODE_AL014 ||
				        	p_dev->alarm_code == __MOTOR_ALARM_CODE_AL015) {
				            m_servo.exception |= (uint16_t)__SVO_EXCEPTION_ROUGH_FB;
		                } else {
                            m_servo.exception &= (uint16_t)~__SVO_EXCEPTION_ROUGH_FB;
                        }
                        break;
                    case PRJCONF_ROUGH_UD:
                        if (p_dev->alarm_code == __MOTOR_ALARM_CODE_AL014 ||
				        	p_dev->alarm_code == __MOTOR_ALARM_CODE_AL015) {
				            m_servo.exception |= (uint16_t)__SVO_EXCEPTION_ROUGH_UD;
		                } else {
                            m_servo.exception &= (uint16_t)~__SVO_EXCEPTION_ROUGH_UD;
                        }
                        break;
                    case PRJCONF_FINE_LR:
                        if (p_dev->alarm_code == __MOTOR_ALARM_CODE_AL014 ||
				        	p_dev->alarm_code == __MOTOR_ALARM_CODE_AL015) {
				            m_servo.exception |= (uint16_t)__SVO_EXCEPTION_FINE_LR;
		                } else {
                            m_servo.exception &= (uint16_t)~__SVO_EXCEPTION_FINE_LR;
                        }
                        break;
                    case PRJCONF_FINE_UD:
                        if (p_dev->alarm_code == __MOTOR_ALARM_CODE_AL014 ||
				        	p_dev->alarm_code == __MOTOR_ALARM_CODE_AL015) {
				            m_servo.exception |= (uint16_t)__SVO_EXCEPTION_FINE_UD;
		                } else {
                            m_servo.exception &= (uint16_t)~__SVO_EXCEPTION_FINE_UD;
                        }
                        break;
                    default:
                        break;
                }
           
				break;
            default:
                break;
        }
    }
    
    switch (p_dev->stat) {
        case MOTOR_STAT_INIT:
            __motor_config_not_store_in_eeprom(p_dev->addr);
            break;
        case MOTOR_STAT_CONFIG_MOTOR_EEPROM:
            __motor_get_do_status(p_dev->addr);
            break;
        case MOTOR_STAT_CHECK_MOTOR_IS_READY:
            __motor_config_di_source(p_dev->addr);
            break;
        case MOTOR_STAT_ENABLE_IO:
            __motor_stop(p_dev->addr);
            break;
        default:
            break;
    }
}

/** \brief �������ݴ��� */
static void __motor_rx_data_handler(bool rx_status, uint8_t *pbuf, uint16_t len)
{
    uint16_t data[PRJCONF_MOTOR_RX_DATA_BUF_SIZE];
    
    mb_exception_t exc = MB_EXC_NONE;
    
    if (rx_status || (!rx_status && len)) {
        exc = mb_master_pdu_unpacking(pbuf, len, data, PRJCONF_MOTOR_RX_DATA_BUF_SIZE);
        
        if (exc != MB_EXC_CRC_ERROR)
            __motor_fsm(__motor_get(pbuf[0]), __motor_get_action(pbuf[1], mb_master_get_addr_cache()), exc, data, pbuf[2] >> 1);
    } else if (!rx_status && !len) {
	    __motor_fsm(__motor_get(mb_master_get_sid_cache()), 
		            __motor_get_action(mb_master_get_fc_cache(), mb_master_get_addr_cache()),
		            MB_EXC_NONE, NULL, 0);
	}
}

/** \brief ��������ʼ����ɺ�������Թ��� */
static void __motor_periodic_work(void *p_context)
{
	int i;
    int iter_grp[] = PRJCONF_MOTOR_ITER_GRP;
	
	struct motor_dev *p_dev;
	
	for (i = 0; i < sizeof(iter_grp)/sizeof(int); ++i) {
		p_dev = &m_servo.motors[iter_grp[i]];
		if (p_dev->addr != PRJCONF_MOTOR_MODBUS_ADDR_NOTUSE && p_dev->stat >= MOTOR_STAT_READY) {
			__motor_get_alarm_code(p_dev->addr);
			__motor_get_pos(p_dev->addr);
			
			NRF_LOG_INFO("axis slave id: %x, pos: %d, alarm code: %d", p_dev->addr, p_dev->position, p_dev->alarm_code);
        }
	}
}

/** \brief ��������Ƶ������Ƿ��ڵ������ */
static bool __motor_check_in_group(enum motor_axis_idx idx)
{
    int iter_grp[] = PRJCONF_MOTOR_ITER_GRP;
	
	int i;
	
	for (i = 0; i < sizeof(iter_grp)/sizeof(int); ++i) {
	    if (idx == iter_grp[i])
			break;
	}
		
    if (MOTOR_MAX_COUNT == i)
		return false;
	else
	    return true;
}


void servo_init(void)
{
    int i;
	
	ret_code_t err_code;
    
    struct motor_dev *p_dev;

    int iter_grp[] = PRJCONF_MOTOR_ITER_GRP;
    
    mb_master_response_handler_register(__motor_rx_data_handler);
    
    for (i = 0; i < sizeof(iter_grp)/sizeof(int); ++i) {
        p_dev = &m_servo.motors[iter_grp[i]];
        if (p_dev->addr != PRJCONF_MOTOR_MODBUS_ADDR_NOTUSE) {
            __motor_config_not_store_in_eeprom(p_dev->addr);
        }
    }
	
	m_servo.stat = SVO_STAT_IDLE;
	
	err_code = app_timer_create(&motor_reading_status_timer, APP_TIMER_MODE_REPEATED, __motor_periodic_work);
	APP_ERROR_CHECK(err_code);
	
	app_timer_start(motor_reading_status_timer, APP_TIMER_TICKS(PRJCONF_MOTOR_READING_STATUS_PERIOD), NULL);
}

bool servo_motor_ctrl(enum motor_axis_idx idx, enum servo_motor_motion motion)
{
    struct motor_dev *p_dev;
	
    if (idx > MOTOR_MAX_COUNT)
        return false;
	
	if (false == __motor_check_in_group(idx))
		return false;
    
    p_dev = &m_servo.motors[idx];
    
    if (p_dev->addr == PRJCONF_MOTOR_MODBUS_ADDR_NOTUSE || p_dev->stat < MOTOR_STAT_READY)
        return false;
    
    switch (motion) {
        case SVO_MOTOR_STOP:
            __motor_stop(p_dev->addr);
            break;
        case SVO_MOTOR_MOVE_TO_LUF:
            (MOTOR_DIR_POL_CW == p_dev->luf_pol) ? __motor_cw(p_dev->addr) : __motor_ccw(p_dev->addr);
            break;
        case SVO_MOTOR_MOVE_TO_RDB:
            (MOTOR_DIR_POL_CW == p_dev->luf_pol) ? __motor_ccw(p_dev->addr) : __motor_cw(p_dev->addr);
            break;
        default:
            break;
    }
    
    return true;
}

bool servo_motor_set_manual_speed(enum motor_axis_idx idx, uint16_t speed)
{
    struct motor_dev *p_dev;
    
    if (idx > MOTOR_MAX_COUNT)
        return false;
	
	if (false == __motor_check_in_group(idx))
		return false;
    
    p_dev = &m_servo.motors[idx];
	
    if (p_dev->addr == PRJCONF_MOTOR_MODBUS_ADDR_NOTUSE || p_dev->stat < MOTOR_STAT_READY)
        return false;
	
	if (speed > p_dev->max_speed)
		speed = p_dev->max_speed;
	
	if (speed != p_dev->manual_speed)
		__motor_set_jog_speed(idx, speed / PRJCONF_MOTOR_SPEED_SCALE);
	
	return true;
}

bool servo_motor_set_auto_speed(enum motor_axis_idx idx, uint16_t speed)
{
    struct motor_dev *p_dev;
    
    if (idx > MOTOR_MAX_COUNT)
        return false;
	
	if (false == __motor_check_in_group(idx))
		return false;
    
    p_dev = &m_servo.motors[idx];
	
    if (p_dev->addr == PRJCONF_MOTOR_MODBUS_ADDR_NOTUSE || p_dev->stat < MOTOR_STAT_READY)
        return false;
	
	if (speed > p_dev->max_speed)
		speed = p_dev->max_speed;
	
	if (speed != p_dev->auto_speed)
	    __motor_set_jog_speed(idx, speed / PRJCONF_MOTOR_SPEED_SCALE);
	
	return true;
}
