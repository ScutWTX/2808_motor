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


/** \brief 电机状态 */
enum motor_stat {
    MOTOR_STAT_INIT = 0,                /**< \brief 电机初始化 */
    MOTOR_STAT_CONFIG_MOTOR_EEPROM,     /**< \brief 配置EEPROM完成 */
    MOTOR_STAT_CHECK_MOTOR_IS_READY,    /**< \brief 检查电机是否就绪完成 */
    MOTOR_STAT_ENABLE_IO,               /**< \brief 使能IO完成 */
    MOTOR_STAT_READY                    /**< \brief 电机就绪 */
};

/** \brief 电机动作 */
enum motor_action {
    MOTOR_ACTION_NONE = 0,              /**< \brief 无动作 */
    MOTOR_ACTION_CONFIG_MOTOR_EEPROM,   /**< \brief 配置EEPROM */
    MOTOR_ACTION_CHECK_MOTOR_IS_READY,  /**< \brief 检查电机是否就绪 */
    MOTOR_ACTION_ENABLE_IO,             /**< \brief 使能IO */
    MOTOR_ACTION_ENABLE,                /**< \brief 使能电机 */
	MOTOR_ACTION_GET_POS,               /**< \brief 获取位置 */
	MOTOR_ACTION_GET_ALARM_CODE,        /**< \brief 获取报警状态 */
	MOTOR_ACTION_SET_SPEED,             /**< \brief 设置速度 */
};

/** \brief 伺服状态 */
enum servo_stat {
    SVO_STAT_IDLE = 0,
    SVO_STAT_WORKING
};

/** \brief 电机下一个状态 */
#define __MOTOR_NEXT_STAT(p_dev) p_dev->stat += 1

/** \brief 电机上一个状态 */
#define __MOTOR_LAST_STAT(p_dev) p_dev->stat -= 1

/** \brief 电机设置状态*/
#define __MOTOR_SET_STAT(p_dev, stat) p_dev->stat = stat

/** \brief 伺服下一个状态 */
#define __SVO_NEXT_STAT(p_svo, stat) p_svo->stat = stat


#define __SVO_EXCEPTION_ROUGH_FB (1 << 0) /**< \brief 粗调前后限位异常 */
#define __SVO_EXCEPTION_ROUGH_UD (1 << 1) /**< \brief 粗调上下限位异常 */
#define __SVO_EXCEPTION_FINE_LR  (1 << 2) /**< \brief 精调左右限位异常 */
#define __SVO_EXCEPTION_FINE_UD  (1 << 3) /**< \brief 精调上下限位异常 */


/** \brief 电机设备 */
struct motor_dev {
    enum motor_stat stat;       /**< \brief 电机状态               */
    enum motor_axis_idx idx;    /**< \brief 轴类型                 */
    enum motor_dir_pol luf_pol; /**< \brief (左、上、前)方向极性   */
    
	uint16_t    max_speed;      /**< \brief 最大速度,单位:(mm/min) */
    uint16_t   auto_speed;      /**< \brief 自动速度,单位:(mm/min) */
    uint16_t manual_speed;      /**< \brief 手动速度,单位:(mm/min) */
	
	uint16_t alarm_code;        /**< \brief 报警代码               */
                                                                 
    int32_t position;           /**< \brief 位置信息               */
                                
    uint8_t addr;               /**< \brief 电机modbus地址         */
};

/** \brief 摆动器参数 */
struct wiggler_params {
    uint16_t left_range;               /**< \brief 摆动器左摆幅    */
    uint16_t left_speed;               /**< \brief 摆动器左摆速度  */
    uint16_t left_stop_time;           /**< \brief 摆动器左停时间  */
    uint16_t right_range;              /**< \brief 摆动器右摆幅    */
    uint16_t right_speed;              /**< \brief 摆动器右摆速度  */
    uint16_t right_stop_time;          /**< \brief 摆动器右停时间  */
    uint16_t mid_stop_time;            /**< \brief 摆动器中停时间  */
    uint16_t start_delay;              /**< \brief 摆动器启动延时  */
    uint16_t stop_delay;               /**< \brief 摆动器停止延时  */
};

/** \brief 伺服设备 */
struct servo {
    struct motor_dev motors[MOTOR_MAX_COUNT]; /**< \brief 电机设备         */
    
    uint16_t start_delay;                     /**< \brief 主运动轴启动延时 */
    uint16_t stop_delay;                      /**< \brief 主运动轴停止延时 */
     
    struct wiggler_params wiggler;            /**< \brief 摆动器           */

    uint16_t exception;                       /**< \brief 异常码           */
    enum servo_stat stat;                     /**< \brief 状态             */
	
	uint32_t last_tick;                       /**< \brief 上一个时间点     */
	uint32_t cur_tick;                        /**< \brief 当前时间点       */
};

/** 伺服控制器 */
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

/** \brief 电机读取定时器 */
APP_TIMER_DEF(motor_reading_status_timer);

/************************ 电机控制 *******************************************/

#define __MOTOR_AUX_FUNC_REG_ADDR 0x023C /**< \brief 辅助机能寄存器 */
#define __MOTOR_AUX_FUNC_VAL      0x0005 /**< \brief 电机参数掉电不保存在eeprom */

/** \brief 配置电机参数断电后不保持(doc: P316) */
static void __motor_config_not_store_in_eeprom(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_AUX_FUNC_VAL;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_AUX_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len);    
}

#define __MOTOR_DO_STAT_REG_ADDR 0x005C /**< \brief 驱动器数字输出(DO)信号状态显示 */
#define __MOTOR_DO_STAT_READY    0x0001 /**< \brief 驱动器就绪时bit0置1 */

/** \brief 获取DO信号状态(doc: P270) */
static void __motor_get_do_status(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_DO_STAT_REG_ADDR, NULL, 1, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_DI_SOURCE_REG_ADDR 0x030C /**< \brief 输入接点(DI)来源控制开关 */
#define __MOTOR_DI_SOURCE_VAL      0x0007 /**< \brief 配置值 */

/** \brief 配置DI来源，使能bit0~bit2，即(DI1~DI3，对应0x040E地址的bit0~bit2)(doc:P338) */
static void __motor_config_di_source(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_SOURCE_VAL;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_SOURCE_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

#define __MOTOR_DI_MULTI_FUNC_REG_ADDR 0x040E /**< \brief 数字接入点多重功能 */
#define __MOTOR_DI_MULTI_FUNC_IDLE     0x0001 /**< \brief 使能(停止) */
#define __MOTOR_DI_MULTI_FUNC_CW       0x0007 /**< \brief 正转 */
#define __MOTOR_DI_MULTI_FUNC_CCW      0x0003 /**< \brief 反转 */

/** \brief 使能电机(停止电机)(doc:P345) */
static void __motor_stop(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_IDLE;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/** \brief 电机正转 */
static void __motor_cw(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_CW;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/** \brief 电机反转 */
static void __motor_ccw(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    uint16_t data = __MOTOR_DI_MULTI_FUNC_CCW;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_DI_MULTI_FUNC_REG_ADDR, &data, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

#define __MOTOR_STAT_MONITOR_CACHE_POS_REG_ADDR 0x0012 /**< \brief 状态监控缓存器(位置信息) 32bits */

/** \brief 获取电机位置信息(doc:P262) */
static void __motor_get_pos(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_STAT_MONITOR_CACHE_POS_REG_ADDR, NULL, 2, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_ALARM_CODE_REG_ADDR 0x0002 /**< \brief 驱动器报警代码 */
#define __MOTOR_ALARM_CODE_AL014    0x14   /**< \brief 反向极限异常 */
#define __MOTOR_ALARM_CODE_AL015    0x15   /**< \brief 正向极限异常 */

/** \brief 获取电机报警代码(doc:P260,P570) */
static void __motor_get_alarm_code(uint8_t sid)
{
    uint8_t __pdu[8];
    uint16_t len;
    
    mb_master_pdu_packing(sid, MB_FC_READ_HOLDING_REGS, __MOTOR_ALARM_CODE_REG_ADDR, NULL, 1, __pdu, &len);
    
    mb_master_request(__pdu, len);
}

#define __MOTOR_JOG_SPEED_REG_ADDR 0x040A /**< \brief JOG(寸动)控制,速度寄存器地址,单位:rpm */

/** \brief 设置速度值 */
static void __motor_set_jog_speed(uint8_t sid, uint16_t speed)
{
    uint8_t __pdu[8];
    uint16_t len;

    mb_master_pdu_packing(sid, MB_FC_WRITE_REG, __MOTOR_JOG_SPEED_REG_ADDR, &speed, 1, __pdu, &len);

    mb_master_request(__pdu, len); 
}

/************************ 摆动器控制 *****************************************/

#define __WIGGLER_LEFT_RANGE_REG_ADDR  0x0636 /**< \brief 摆动器左摆幅寄存器地址   */
#define __WIGGLER_LEFT_SPEED_REG_ADDR  0x0590 /**< \brief 摆动器左摆速度寄存器地址 */
#define __WIGGLER_LEFT_DELAY_REG_ADDR  0x056C /**< \brief 摆动器左停时间寄存器地址 */
#define __WIGGLER_RIGHT_RANGE_REG_ADDR 0x062E /**< \brief 摆动器右摆幅寄存器地址   */
#define __WIGGLER_RIGHT_SPEED_REG_ADDR 0x058E /**< \brief 摆动器右摆速度寄存器地址 */
#define __WIGGLER_RIGHT_DELAY_REG_ADDR 0x056E /**< \brief 摆动器右停时间寄存器地址 */


/** \brief 通过电机从机地址找到对应的电机设备 */
struct motor_dev *__motor_get(uint8_t sid)
{
    int i;
    
    for (i = 0; i < MOTOR_MAX_COUNT; ++i) {
        if (sid == m_servo.motors[i].addr)
            return &m_servo.motors[i];
    }
    
    return NULL;
}

/** \brief 根据功能码和操作地址判断该次请求为哪个动作 */
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

/** \brief 电机状态机 */
static void __motor_fsm(struct motor_dev *p_dev, enum motor_action action, mb_exception_t action_ret, uint16_t *p_data, uint16_t len)
{
    if (NULL == p_dev)
        return;

    //处理动作所产生的数据并决定是否跳转到下一个状态
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

/** \brief 接收数据处理 */
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

/** \brief 驱动器初始化完成后的周期性工作 */
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

/** \brief 检查所控制的轴电机是否在电机组中 */
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
