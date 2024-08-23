#include "nrfx_timer.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "prjconfig.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"

#include "encoder.h"

/** \brief 编码器计数器 */
const nrf_drv_timer_t counter_encoder_timer = NRF_DRV_TIMER_INSTANCE(2);

/** \brief 编码器采样定时器 */
APP_TIMER_DEF(encoder_sample_timer);

static struct encoder_dev m_encoder;

/** \brief 计数采样定时器超时回调函数 */
static void __encoder_sample_expired(void *p_context)
{
    static int cnt = 0;
    encoder_handle_t handle = (encoder_handle_t)p_context;
    
    handle->cur_capture_cnt = nrfx_timer_capture(&counter_encoder_timer, NRF_TIMER_CC_CHANNEL0);
    
    handle->freq_hz = (handle->cur_capture_cnt - handle->last_capture_cnt) * 1000 / PRJCONF_DRV8432_ENCODER_SAMPLE_TIME;
    
    if (!handle->freq_hz) {
        handle->dir = ENCODER_DIR_NONE;
    }
    
    handle->rpm = handle->freq_hz * 60 * 10 / PRJCONF_DRV8432_ENCODER_RESOLUTION / PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO;
    handle->wire_speed = (uint32_t)((double)handle->rpm * 3.14 * PRJCONF_DRV8432_MOTOR_DIAMETER);

    if (cnt++ == 20) {
        cnt = 0;
        NRF_LOG_INFO("freq:%d, dir: %s, speed(0.1rpm):%d, speed(0.1mm/min):%d", handle->freq_hz, handle->dir == ENCODER_DIR_CW ? "CW" : (handle->dir == ENCODER_DIR_CCW ? "CCW" : "NONE"), handle->rpm, handle->wire_speed);
    }
    
    handle->last_capture_cnt = handle->cur_capture_cnt;
}

static void __pin_a_gpiote_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//    NRF_LOG_INFO("pin:%d, action:%x", pin, action);
    if (pin == m_encoder.pin_enc_a && GPIOTE_CONFIG_POLARITY_LoToHi == action)
        m_encoder.dir = ENCODER_DIR_CCW;
}

static void __pin_b_gpiote_evt_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
//    NRF_LOG_INFO("pin:%d, action:%x", pin, action);
    if (pin == m_encoder.pin_enc_b && GPIOTE_CONFIG_POLARITY_LoToHi == action)
        m_encoder.dir = ENCODER_DIR_CW;
}

/** \brief 定时器配置 */
static ret_code_t __timer_config(struct encoder_dev *encoder)
{
    ret_code_t err_code = NRF_SUCCESS;
    
    nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_cfg.mode = NRF_TIMER_MODE_COUNTER;
    
    err_code = nrfx_timer_init(&counter_encoder_timer, &timer_cfg, NULL);
    
    return err_code;
}

/** \brief 引脚输入事件配置 */
static ret_code_t __enc_pin_gpiote_config(struct encoder_dev *encoder)
{
    ret_code_t err_code = NRF_SUCCESS;
    
    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
            return NRF_ERROR_INTERNAL;
    }

    nrfx_gpiote_in_config_t in_cfg = GPIOTE_CONFIG_IN_SENSE_LOTOHI(false);
    
    err_code = nrf_drv_gpiote_in_init(encoder->pin_enc_a, &in_cfg, __pin_a_gpiote_evt_handler);
    if (err_code != NRF_SUCCESS)
        return NRF_ERROR_NO_MEM;

    err_code = nrf_drv_gpiote_in_init(encoder->pin_enc_b, &in_cfg, __pin_b_gpiote_evt_handler);
    if (err_code != NRF_SUCCESS)
        return NRF_ERROR_NO_MEM;
    
    return err_code;
}

/** \brief PPI配置 */
static ret_code_t __ppi_config(struct encoder_dev *encoder)
{
    ret_code_t err_code = NRF_SUCCESS;
    
    err_code = nrf_drv_ppi_init();
    if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_MODULE_ALREADY_INITIALIZED))
        return NRF_ERROR_NO_MEM;
    
    if (nrf_drv_ppi_channel_alloc(&encoder->channel_a) != NRF_SUCCESS)
        return NRF_ERROR_NO_MEM; 
    
    if (nrf_drv_ppi_channel_alloc(&encoder->channel_b) != NRF_SUCCESS)
        return NRF_ERROR_NO_MEM;    
    
    nrf_drv_ppi_channel_disable(encoder->channel_a);
    nrf_drv_ppi_channel_disable(encoder->channel_b);
    
    err_code = nrfx_ppi_channel_assign(encoder->channel_a,
                                       nrfx_gpiote_in_event_addr_get(encoder->pin_enc_a),
                                       nrfx_timer_task_address_get(&counter_encoder_timer, NRF_TIMER_TASK_COUNT));
    if (err_code != NRF_SUCCESS)
        return err_code;
    
    err_code = nrfx_ppi_channel_assign(encoder->channel_b,
                                       nrfx_gpiote_in_event_addr_get(encoder->pin_enc_b),
                                       nrfx_timer_task_address_get(&counter_encoder_timer, NRF_TIMER_TASK_COUNT));
    if (err_code != NRF_SUCCESS)
        return err_code;
    
    return err_code;
}

/** \brief 编码器初始化 */
static void __encoder_init(struct encoder_dev *encoder)
{
    ret_code_t err_code;
    
    err_code = __timer_config(encoder);
    if (err_code != NRF_SUCCESS)
        return;
    
    err_code = __enc_pin_gpiote_config(encoder);
    if (err_code != NRF_SUCCESS)
        return;
    
    err_code = __ppi_config(encoder);
    if (err_code != NRF_SUCCESS)
        return;

    /* 使能定时器 */
    nrf_drv_timer_enable(&counter_encoder_timer);
    
    /* 使能PPI通道 */
    nrfx_ppi_channel_enable(encoder->channel_a);  
    nrfx_ppi_channel_enable(encoder->channel_b);   
    
    /* 使能GPIOTE输入事件 */
    nrfx_gpiote_in_event_enable(encoder->pin_enc_a, true);
    nrfx_gpiote_in_event_enable(encoder->pin_enc_b, true);
}

/** \brief 编码器初始化 */
encoder_handle_t encoder_init(uint32_t enc_a_pin, uint32_t enc_b_pin)
{
    ret_code_t err_code;
    
    m_encoder.pin_enc_a = enc_a_pin;
    m_encoder.pin_enc_b = enc_b_pin;
    
    m_encoder.last_capture_cnt = 0;
    m_encoder.cur_capture_cnt  = 0;
    
    m_encoder.dir = ENCODER_DIR_NONE;
    
    __encoder_init(&m_encoder);
    
    err_code = app_timer_create(&encoder_sample_timer, APP_TIMER_MODE_REPEATED, __encoder_sample_expired);
	APP_ERROR_CHECK(err_code);
    
    app_timer_start(encoder_sample_timer, APP_TIMER_TICKS(PRJCONF_DRV8432_ENCODER_SAMPLE_TIME), &m_encoder);
    
    return &m_encoder;
}


enum encoder_dir encoder_get_dir(encoder_handle_t handle)
{
    return handle->dir;
}

uint32_t encoder_get_freq(encoder_handle_t handle)
{
    return handle->freq_hz;
}

uint32_t encoder_get_rpm(encoder_handle_t handle)
{
    return handle->rpm;
}

uint16_t encoder_get_wire_speed(encoder_handle_t handle)
{
    return handle->wire_speed / 10;
}
