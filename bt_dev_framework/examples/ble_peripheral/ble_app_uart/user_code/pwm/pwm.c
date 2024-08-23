#include "nrf_gpio.h" 


#include "pwm.h"

#include "prjconfig.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define __PWM_FREQ_2_US(freq) ((uint32_t)(1000000UL / (freq)))

/** \brief 使用TIMER1创建PWM实例PWM0 */
APP_PWM_INSTANCE(PWM0, 1);

/** \brief PWM设备 */
static pwm_dev_t m_pwm;

/** \brief PWM就绪状态标志 */
static volatile bool __pwm0_ready_flag;

/** \brief PWM回调函数 */
static void __pwm_ready_callback(uint32_t pwm_id)
{
    __pwm0_ready_flag = true;
}

pwm_handle_t pwm_init(uint32_t freq_hz, uint32_t pwm_a_pin, uint32_t pwm_b_pin)
{
    ret_code_t err_code;
    
    app_pwm_config_t pwm0_cfg = APP_PWM_DEFAULT_CONFIG_2CH(__PWM_FREQ_2_US(freq_hz), pwm_a_pin, pwm_b_pin);
    
    pwm0_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    pwm0_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;
    
    err_code = app_pwm_init(&PWM0, &pwm0_cfg, __pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    
    m_pwm.freq_hz    = freq_hz;
    m_pwm.pwm_a_pin  = pwm_a_pin;
    m_pwm.pwm_b_pin  = pwm_b_pin;
    m_pwm.pwm_handle = &PWM0;
    
    app_pwm_enable(m_pwm.pwm_handle);
    
    return &m_pwm;
}

void pwm_output_stop(pwm_handle_t handle)
{
    app_pwm_disable(handle->pwm_handle);
}

void pwm_output_cw(pwm_handle_t handle, uint16_t duty_x10)
{
    int i = 1000;
    
    if (duty_x10 == 0 || duty_x10 > 1000) //占空比不能大于100%
        return;
    
    app_pwm_enable(handle->pwm_handle);
    
    __pwm0_ready_flag = false;
    
    app_pwm_channel_duty_x10_set(handle->pwm_handle, 0, duty_x10);
    
    while (!__pwm0_ready_flag && i--);
    
    app_pwm_channel_duty_set(handle->pwm_handle, 1, 0);
}

void pwm_output_ccw(pwm_handle_t handle, uint16_t duty_x10)
{
    int i = 1000;
    
    if (duty_x10 == 0 || duty_x10 > 1000) //占空比不能大于100%
        return;
    
    app_pwm_enable(handle->pwm_handle);
    
    __pwm0_ready_flag = false;
    
    app_pwm_channel_duty_set(handle->pwm_handle, 0, 0);
    
    while (!__pwm0_ready_flag && i--);
    
    app_pwm_channel_duty_x10_set(handle->pwm_handle, 1, duty_x10);
}
