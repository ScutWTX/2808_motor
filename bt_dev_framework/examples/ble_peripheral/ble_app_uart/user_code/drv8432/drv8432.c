#include "drv8432.h"
#include "nrf_delay.h"
#include "prjconfig.h"

static drv8432_dev_t m_drv8432;


drv8432_handle_t drv8432_init(void)
{
    m_drv8432.reset_ab_pin = PRJCONF_DRV8432_RESETAB_PIN;
    m_drv8432.reset_cd_pin = PRJCONF_DRV8432_RESETCD_PIN;
    
    m_drv8432.otw_pin      = PRJCONF_DRV8432_OTW_PIN;
    m_drv8432.fault_pin    = PRJCONF_DRV8432_FAULT_PIN;
    
    nrf_gpio_cfg_output(m_drv8432.reset_ab_pin);
    nrf_gpio_cfg_output(m_drv8432.reset_cd_pin);
    
    nrf_gpio_cfg_input(m_drv8432.otw_pin,   NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(m_drv8432.fault_pin, NRF_GPIO_PIN_PULLUP);
    
    m_drv8432.pwm_handle     = pwm_init(PRJCONF_DRV8432_PWM_FREQ, PRJCONF_DRV8432_PWMA_PIN, PRJCONF_DRV8432_PWMB_PIN);
    m_drv8432.encoder_handle = encoder_init(PRJCONF_DRV8432_ENC_A_PIN, PRJCONF_DRV8432_ENC_B_PIN);
    
    drv8432_reset(&m_drv8432);
    
    return &m_drv8432;
}

void drv8432_reset(drv8432_handle_t handle)
{
    nrf_gpio_pin_clear(handle->reset_ab_pin);
    nrf_gpio_pin_clear(handle->reset_cd_pin);
    
    nrf_delay_ms(PRJCONF_DRV8432_RESET_HOLDING_TIME);
    
    nrf_gpio_pin_set(handle->reset_ab_pin);
    nrf_gpio_pin_set(handle->reset_cd_pin);
}

bool drv8432_fault_stat(drv8432_handle_t handle)
{
    return nrf_gpio_pin_read(handle->fault_pin) ? false : true;
}

bool drv8432_otw_stat(drv8432_handle_t handle)
{
    return nrf_gpio_pin_read(handle->otw_pin) ? false : true; 
}

uint16_t drv8432_get_encoder_wire_speed(drv8432_handle_t handle)
{
    return encoder_get_wire_speed(handle->encoder_handle);
}
