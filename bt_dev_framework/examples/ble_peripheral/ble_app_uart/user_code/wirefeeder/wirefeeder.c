#include "wirefeeder.h" 
#include "drv8432.h"
#include "app_timer.h"
#include "prjconfig.h"

#include "led.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "event.h"
#include "event_pool.h"

/** \brief 分段中时间参数最大值9999,即999.9s,表示无限长的时间 */
#define __MAX_SEG_TIME_VALUE      (9999) 

/**< \brief 分段渐变阶段插补定时时间(ms) */
#define __SEG_SLOPE_INTERVAL_TIME (50)


//#define __WFR_PID_P (0.8)
//#define __WFR_PID_I (0.2)
//#define __WFR_PID_D (0)

#define __WFR_PID_CTRL_PERIOD (50) //PID控制周期ms

#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_GNL_32_21E

#define __WFR_PID_P (0.5)
#define __WFR_PID_I (0.1)
#define __WFR_PID_D (0.1)

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_0

#define __WFR_PID_P (0.5)
#define __WFR_PID_I (0.1)
#define __WFR_PID_D (0.1)

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1

#define __WFR_PID_P (0.9)
#define __WFR_PID_I (0.1)
#define __WFR_PID_D (0)

#else

#define __WFR_PID_P (0.9)
#define __WFR_PID_I (0.1)
#define __WFR_PID_D (0)

#endif

/** \brief import from main.c */
extern event_loop_t el;


/** \brief 设备启动接口 */
static void __dev_start_if(void);

/** \brief 设备停止接口 */
static void __dev_stop_if(void);

/** \brief 段切换 */
static void __seg_switch(void);

/** \brief 速度设置接口 */
static void __speed_setting_if(int16_t speed);



APP_TIMER_DEF(wfr_pull_timer);

APP_TIMER_DEF(wfr_periodic_work_timer);
APP_TIMER_DEF(seg_proc_timer);
APP_TIMER_DEF(seg_interval_timer);

struct wfr_dev {
    volatile bool is_startup;               /**< \brief 启停标志, true-启动,false-停止     */
    volatile bool is_auto_pulling;          /**< \brief 停止抽丝标志, true-有效,false-无效 */
                                    
    drv8432_handle_t drv8432;               /**< \brief DRV8432设备句柄                    */
    
    volatile enum wfr_dir dir;              /**< \brief 送丝方向                           */
    volatile uint16_t current_out_speed;    /**< \brief 当前设定值                         */
    volatile uint16_t push_speed;           /**< \brief 送丝速度                           */
    volatile uint16_t pull_speed;           /**< \brief 抽丝速度,只有在停止后抽丝时使用    */
    volatile uint16_t pull_time;            /**< \brief 抽丝时间                           */
    volatile enum wfr_status status;        /**< \brief 状态码                             */
    
    event_t seg_switch_event;
    
    struct { 
        volatile uint16_t pid_output;       /**< \brief PID输出(速度值)                */
        int16_t prev_err;                    /**< \brief 前一个偏差量                   */ 
        int16_t prev_prev_err;               /**< \brief 前前一个偏差量                 */
    } pid_params;
    
    seg_manager seg;                        /**< \brief 分段管理                           */
};

static struct wfr_dev m_wfr;


static void __pid_params_clean(void)
{
    m_wfr.pid_params.pid_output    = 0;
    m_wfr.pid_params.prev_err      = 0;
    m_wfr.pid_params.prev_prev_err = 0;
}

static bool __pid_calculate(uint32_t target_value, uint32_t measured_value, double Kp, double Ki, double Kd, volatile uint16_t *p_output)
{
    double pid_delta = 0.0f;
    
    int16_t curr_err = 0.0;
    
    bool ret = false;
    
//    uint16_t deviation = 0;
    
    int16_t output = (int16_t)*p_output;

	curr_err = target_value - measured_value;
    
//    deviation = target_value / 10;
    
//    if (curr_err < -deviation || curr_err > deviation) {
    if (curr_err != 0) {
    
        pid_delta = Kp *  (curr_err - m_wfr.pid_params.prev_err) + 
                    Ki *   curr_err + 
                    Kd * ((curr_err - m_wfr.pid_params.prev_err) - (m_wfr.pid_params.prev_err - m_wfr.pid_params.prev_prev_err));
        
//        NRF_LOG_INFO("peror:%d", (int16_t)(Kp * (curr_err - m_wfr.pid_params.prev_err)));

//        NRF_LOG_INFO("pid_delta: %d, curr_err:%d, prev_err:%d, prev_prev_err:%d", (int16_t)pid_delta, (int16_t)curr_err, (int16_t)m_wfr.pid_params.prev_err, (int16_t)m_wfr.pid_params.prev_prev_err);
        
        output += (int16_t)pid_delta;
        
//        NRF_LOG_INFO("target_value:%d, measured_value:%d, pid_delta: %d, pid_output:%d", target_value, measured_value, (int16_t)pid_delta, output);

	    if (output > PRJCONF_WFR_MAX_SPEED) output = PRJCONF_WFR_MAX_SPEED;
	    if (output < PRJCONF_WFR_MIN_SPEED) output = PRJCONF_WFR_MIN_SPEED;

        *p_output = (uint16_t)output;
        
        m_wfr.pid_params.prev_prev_err = m_wfr.pid_params.prev_err;
        m_wfr.pid_params.prev_err      = curr_err;

//        NRF_LOG_INFO("target_value:%d, measured_value:%d, pid_delta: %d, pid_output:%d", target_value, measured_value, (int16_t)pid_delta*1000, *p_output);
        
        ret = true;
    } else {
        m_wfr.pid_params.prev_prev_err = m_wfr.pid_params.prev_err;
        m_wfr.pid_params.prev_err      = curr_err;
        
        ret = false; 
    }
    
    return ret;
}

/** \brief 送丝速度转换到PWM占空比(0.1%) */
static uint16_t __wfr_speed_2_pwm_duty_x10(uint16_t speed)
{
    uint32_t _speed = speed;
    
    return (uint16_t)(_speed * PRJCONF_WFR_SPEED_2_PWM_LINEAR_A / 10000 + PRJCONF_WFR_SPEED_2_PWM_LINEAR_B);
}

/** \brief 抽丝时间到则停止抽丝 */
static void __wfr_pull_timeout_handler(void *p_context)
{
    pwm_output_stop(m_wfr.drv8432->pwm_handle);
    m_wfr.is_auto_pulling = false;
}

/** \brief 抽丝 */
static void __wfr_rollback(void)
{
    uint16_t duty_x10;
    
    if (m_wfr.pull_speed && m_wfr.pull_time) {
        duty_x10 = __wfr_speed_2_pwm_duty_x10(m_wfr.pull_speed);
        
        (PRJCONF_WFR_POLARITY_OF_PUSH == WFR_MOTOR_CW) ? pwm_output_ccw(m_wfr.drv8432->pwm_handle, duty_x10) : pwm_output_cw(m_wfr.drv8432->pwm_handle, duty_x10);
        
        app_timer_start(wfr_pull_timer, APP_TIMER_TICKS(m_wfr.pull_time * 100), NULL);
    }
}

/** \brief 进丝 */
static void __wfr_push(uint16_t speed)
{
    uint16_t duty_x10 = 0;
    
    m_wfr.current_out_speed = speed;
    
    if (!speed) {
        pwm_output_stop(m_wfr.drv8432->pwm_handle);
        return;
    }
    
    duty_x10 = __wfr_speed_2_pwm_duty_x10(speed);
    
//    duty_x10 = speed;
    
    NRF_LOG_INFO("duty_x10:%d", duty_x10);
    
   (PRJCONF_WFR_POLARITY_OF_PUSH == WFR_MOTOR_CW) ? pwm_output_cw(m_wfr.drv8432->pwm_handle, duty_x10) : pwm_output_ccw(m_wfr.drv8432->pwm_handle, duty_x10);
}

/** \brief 退丝 */
static void __wfr_pull(uint16_t speed)
{
    uint16_t duty_x10 = 0;
    
    m_wfr.current_out_speed = speed;
    
    if (!speed) {
        pwm_output_stop(m_wfr.drv8432->pwm_handle);
        return;
    }
    
    duty_x10 = __wfr_speed_2_pwm_duty_x10(speed);
    
    (PRJCONF_WFR_POLARITY_OF_PUSH == WFR_MOTOR_CW) ? pwm_output_ccw(m_wfr.drv8432->pwm_handle, duty_x10) : pwm_output_cw(m_wfr.drv8432->pwm_handle, duty_x10);
}

/** \brief 送丝机速度调整 */
static void __wfr_periodic_work_handler(void *p_context)
{
    bool valid = false;
    
    if (drv8432_fault_stat(m_wfr.drv8432) || drv8432_otw_stat(m_wfr.drv8432)) {
//        drv8432_reset(m_wfr.drv8432);
    } else {
        if (m_wfr.is_startup) {
            valid = __pid_calculate(m_wfr.push_speed, drv8432_get_encoder_wire_speed(m_wfr.drv8432), __WFR_PID_P, __WFR_PID_I, __WFR_PID_D, &m_wfr.pid_params.pid_output);
            if (valid == true) {
                if (m_wfr.dir == WFR_DIR_PUSH) {
                    __wfr_push(m_wfr.pid_params.pid_output);
                } else {
                    __wfr_pull(m_wfr.pid_params.pid_output);
                }
            }
        }
    }
}

/** \brief 分段区间定时处理 */
static void __seg_proc_timer_handler(void *p_context)
{
    uint8_t flag = (uint8_t)p_context;
    uint16_t idx = 0, drawing_speed = 0, drawing_time = 0;
    
    idx = m_wfr.seg.max_seg_id - m_wfr.seg.cur_seg - 1;
    
    drawing_speed = m_wfr.seg.data[idx].drawing_speed;
    drawing_time  = m_wfr.seg.data[idx].drawing_time;
    
    NRF_LOG_INFO("seg proc, idx = %d", idx);
    
    if (TIMING_OF_FEEDING == flag) {
        if (drawing_time) {
            if (drawing_time != __MAX_SEG_TIME_VALUE)
                app_timer_start(seg_proc_timer, APP_TIMER_TICKS(drawing_time * 100), (void *)TIMING_OF_DRAWING);
            
            if (drawing_speed)
                __speed_setting_if((int16_t)-drawing_speed);
        } else {
//            m_wfr.seg.cur_seg++;
//            if (m_wfr.seg.cur_seg <= m_wfr.seg.max_seg_id - 1) {
//                __seg_switch();
//            } else {
//                __dev_stop_if();
//            }
//            event_loop_event_post(&el, &m_wfr.seg_switch_event);
        }
    } else if (TIMING_OF_DRAWING == flag) {
//        m_wfr.seg.cur_seg++;
//        if (m_wfr.seg.cur_seg <= m_wfr.seg.max_seg_id - 1) {
//            __seg_switch();
//        } else {
//            __dev_stop_if();
//        }
//        event_loop_event_post(&el, &m_wfr.seg_switch_event);
    }
}

/** \brief 分段渐变阶段定时处理 */
static void __seg_interval_timer_handler(void *p_context)
{
    m_wfr.seg.cur_val += m_wfr.seg.intval;
    __speed_setting_if(m_wfr.seg.cur_val);
}

static void seg_switch_event_handler(void *p_context, event_t *e)
{
    seg_switch();
}

void wfr_init(void)
{
    ret_code_t err_code;
    
    m_wfr.is_startup      = false;
    m_wfr.is_auto_pulling = false;
    
    m_wfr.drv8432 = drv8432_init();
    
    m_wfr.dir = WFR_DIR_PUSH;
    
    m_wfr.push_speed = 0;
    m_wfr.pull_speed = 0;
    m_wfr.pull_time  = 0;
    m_wfr.status     = WFR_STAT_OK;
    
    err_code = app_timer_create(&wfr_pull_timer, APP_TIMER_MODE_SINGLE_SHOT, __wfr_pull_timeout_handler);
	APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&wfr_periodic_work_timer, APP_TIMER_MODE_REPEATED, __wfr_periodic_work_handler);
	APP_ERROR_CHECK(err_code);
    
    app_timer_create(&seg_proc_timer,     APP_TIMER_MODE_SINGLE_SHOT, __seg_proc_timer_handler);
    app_timer_create(&seg_interval_timer, APP_TIMER_MODE_REPEATED,    __seg_interval_timer_handler);
    
//    app_timer_start(wfr_periodic_work_timer, APP_TIMER_TICKS(__WFR_PID_CTRL_PERIOD), NULL);

    event_init(&m_wfr.seg_switch_event, seg_switch_event_handler, NULL, EVT_MAKE_PRIORITY(EVT_HIGHEST_GRP_PRIO, 1));
}

void wfr_set_rollback_when_stop(bool is_active)
{
    NRF_LOG_INFO("wfr is auto pulling:%d", is_active);
    m_wfr.is_auto_pulling = is_active;
    
//    led_on();
    
    if (m_wfr.is_startup) {
        switch (m_wfr.dir) {
            case WFR_DIR_PUSH:
                __wfr_push(m_wfr.push_speed);
                break;
            case WFR_DIR_PULL:
                __wfr_pull(m_wfr.push_speed);
                break;
            default:
                break;
        }
//#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1      
        app_timer_start(wfr_periodic_work_timer, APP_TIMER_TICKS(__WFR_PID_CTRL_PERIOD), NULL);
        m_wfr.pid_params.pid_output = m_wfr.push_speed;
//#endif
    } else {
        pwm_output_stop(m_wfr.drv8432->pwm_handle);
        
//        led_off();
        
//#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1            
        app_timer_stop(wfr_periodic_work_timer);
        __pid_params_clean();
//#endif
        
        if (m_wfr.is_auto_pulling)
            __wfr_rollback();
    }
}

void wfr_set_dir(enum wfr_dir dir)
{
    NRF_LOG_INFO("wfr dir:%d", dir);
    m_wfr.dir = dir;
}

void wfr_startup(bool is_startup)
{
    if (m_wfr.is_startup && is_startup)
        return;
    
    NRF_LOG_INFO("wfr startup:%d", is_startup);
    
    m_wfr.is_startup = is_startup;
}

void wfr_set_push_speed(uint16_t speed)
{   
    //speed *= 2;
    
    NRF_LOG_INFO("wfr push speed:%d", speed);

    if (speed > PRJCONF_WFR_MAX_SPEED)
        speed = PRJCONF_WFR_MAX_SPEED;
    
    if (m_wfr.is_startup && m_wfr.dir == WFR_DIR_PUSH) {
        __wfr_push(speed);
//        m_wfr.pid_params.pid_output = speed;
    } else if (m_wfr.dir == WFR_DIR_PULL) {
        __wfr_pull(speed);
    }
    
    m_wfr.push_speed = speed;
}

void wfr_set_pull_speed(uint16_t speed)
{
    NRF_LOG_INFO("wfr pull speed:%d", speed);
    
    if (speed > PRJCONF_WFR_MAX_SPEED)
        speed = PRJCONF_WFR_MAX_SPEED;
    
    m_wfr.pull_speed = speed;
}

void wfr_set_pull_time(uint16_t time)
{
    NRF_LOG_INFO("wfr time:%d", time);
    m_wfr.pull_time = time;
}

enum wfr_status wfr_get_status(void)
{
    return m_wfr.status;
}

void set_seg_max_count(uint16_t id)
{
    m_wfr.seg.max_seg_id = id;
}

void set_seg_data(uint16_t id, uint16_t offs, uint16_t data)
{
    *((uint16_t *)(&m_wfr.seg.data[id - 1].id + offs)) = data;
}

void reset_seg(void)
{
    m_wfr.seg.cur_seg = 0;
}

void seg_switch(void)
{
    m_wfr.seg.cur_seg++;
    if (m_wfr.seg.cur_seg <= m_wfr.seg.max_seg_id - 1) {
        __seg_switch();
    } else {
        __dev_stop_if();
    }
}


/** \brief 开始分段过程 */
void start_seg_proc(void);

/** \brief 停止分段过程 */
void stop_seg_proc(void);


/** \brief 分段控制 */
void seg_ctrl(bool startup)
{
    if (startup == true) {
        start_seg_proc();
    } else {
        stop_seg_proc();
    } 
}

/** \brief 开始分段过程 */
void start_seg_proc(void)
{
    m_wfr.seg.cur_seg = 0;
    m_wfr.seg.cur_val = 0;
    
    __dev_start_if();
    
    __seg_switch();
}

/** \brief 停止分段过程 */
void stop_seg_proc(void)
{
    if (m_wfr.seg.max_seg_id != 0 && m_wfr.seg.cur_seg <= m_wfr.seg.max_seg_id - 1) {
        m_wfr.is_startup = true;
        m_wfr.seg.cur_seg++;
        __seg_switch();
    } else {
        app_timer_stop(seg_proc_timer);
        app_timer_stop(seg_interval_timer);
        __dev_stop_if();
    }
}

/** \brief 段切换 */
static void __seg_switch(void)
{ 
    uint16_t idx = 0, feeding_speed = 0, feeding_time = 0, drawing_speed = 0, drawing_time = 0;
    
    if (m_wfr.seg.max_seg_id == 0)
        return;
    
    idx = m_wfr.seg.max_seg_id - m_wfr.seg.cur_seg - 1;
    
    feeding_speed = m_wfr.seg.data[idx].feeding_speed;
    feeding_time  = m_wfr.seg.data[idx].feeding_time;
    drawing_speed = m_wfr.seg.data[idx].drawing_speed;
    drawing_time  = m_wfr.seg.data[idx].drawing_time;
     
    if (feeding_time) {
        if (feeding_time != __MAX_SEG_TIME_VALUE)
            app_timer_start(seg_proc_timer, APP_TIMER_TICKS(feeding_time * 100), (void *)TIMING_OF_FEEDING);
        
        if (feeding_speed)
            __speed_setting_if(feeding_speed);
    } else {
        if (drawing_time) {
            if (drawing_time != __MAX_SEG_TIME_VALUE)
                app_timer_start(seg_proc_timer, APP_TIMER_TICKS(drawing_time * 100), (void *)TIMING_OF_DRAWING);
            
            if (drawing_speed)
                __speed_setting_if((int16_t)-drawing_speed);
        } else {
            m_wfr.seg.cur_seg++;
            __seg_switch();
        }
    }
}


/** \brief 速度设置接口 */
static void __speed_setting_if(int16_t speed)
{
    if (speed > 0) {
        m_wfr.dir = WFR_DIR_PUSH;
    } else if (speed < 0) {
        m_wfr.dir = WFR_DIR_PULL;
    }
    
    if (m_wfr.is_startup) {
        switch (m_wfr.dir) {
            case WFR_DIR_PUSH:
                NRF_LOG_INFO("push speed:%d", speed);
                __wfr_push(speed);
                break;
            case WFR_DIR_PULL:
                NRF_LOG_INFO("pull speed:%d", -speed);
                __wfr_pull(-speed);
                break;
            default:
                break;
        }
    } else {
        pwm_output_stop(m_wfr.drv8432->pwm_handle);
    }
}

/** \brief 设备启动接口 */
static void __dev_start_if(void)
{
    m_wfr.is_startup = true;
}

/** \brief 设备停止接口 */
static void __dev_stop_if(void)
{
    m_wfr.is_startup = false;
    pwm_output_stop(m_wfr.drv8432->pwm_handle);
}
