#include "nrf_gpio.h"
#include "app_timer.h"

#include "prjconfig.h"

/** \brief led��˸��ʹ�õĶ�ʱ�� */
APP_TIMER_DEF(comm_led_timer);


/** \brief ��ʱ���ص����� */
static void __comm_led_timer_expired_cb(void *p_context)
{
    PRJCONF_COMM_LED_ON;
}

/** \brief led��ʼ�� */
void led_init(void)
{
    ret_code_t err_code;
	
	err_code = app_timer_create(&comm_led_timer, APP_TIMER_MODE_SINGLE_SHOT, __comm_led_timer_expired_cb);
	APP_ERROR_CHECK(err_code);
    
    nrf_gpio_cfg_output(PRJCONF_COMM_LED_PIN);
    
    PRJCONF_COMM_LED_ON;
}

/** \brief led�� */
void led_on(void)
{
    PRJCONF_COMM_LED_ON;
}

/** \brief led�� */
void led_off(void)
{
    PRJCONF_COMM_LED_OFF;
}

/** \brief led��˸һ�� */
void led_blink(void)
{
    ret_code_t err_code;
    
	PRJCONF_COMM_LED_OFF;
    
	err_code=app_timer_start(comm_led_timer, APP_TIMER_TICKS(PRJCONF_COMM_LED_LIGHTOFF_TIME), NULL);
    
	APP_ERROR_CHECK(err_code);
}
