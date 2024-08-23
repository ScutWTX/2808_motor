/**
 * \file
 * \brief uart header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */

#include "nrf_gpio.h"
#include "app_uart.h"
#include "prjconfig.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_timer.h"
#include "ringbuf.h"
#include "mb_common.h"
#include "mb_master_uart.h"

#include "led.h"

/** \brief ����״̬ */
enum mb_master_state {
    MB_MASTER_STATE_IDLE,      /**< \brief ����״̬ */
    MB_MASTER_STATE_SILENCE,   /**< \brief ��Ĭ״̬ */
    MB_MASTER_STATE_SENDING,   /**< \brief ����״̬ */
    MB_MASTER_STATE_RECEIVING  /**< \brief ����״̬ */
};

/** \brief �����豸 */
struct mb_master_uart_dev {
    enum mb_master_state state;                /**< \brief ״̬ */
    
    struct ringbuf tx_high_prio_buf;           /**< \brief �����ȼ����ͻ����� */
    struct ringbuf tx_low_prio_buf;            /**< \brief �����ȼ����ͻ����� */
    
	uint8_t  sid_cache;                        /**< \brief �ӻ���ַ���� */
	uint8_t  fc_cache;                         /**< \brief �����뻺�� */
    uint16_t addr_cache;                       /**< \brief ������ʱ���ݴ��ַ */
	

    pfn_recv_data_handler rx_handler;          /**< \brief �û�ע��Ĵ����� */
    bool      rx_status;                       /**< \brief ����״̬��true�����յ�һ֡���ݣ�false���ӻ�Ӧ��ʱ */
    
    uint32_t rx_last_tick;                     /**< \brief ��һ�ν��յ�����ʱ��ʱ��� */
    uint32_t rx_cur_tick;                      /**< \brief ��ǰ���յ�����ʱ��ʱ���   */
    bool     rx_timeout_check;                 /**< \brief ���ճ�ʱ���               */
    
    uint32_t response_last_tick;               /**< \brief ��Ӧ��ʱ��ʱ����һ�λ�ȡ����ʱ��� */
    uint32_t response_cur_tick;                /**< \brief ��Ӧ��ʱ��ʱ�е�ǰ��ȡ����ʱ���   */
    bool     response_timeout_check;           /**< \brief ��Ӧ��ʱ��� */

    uint8_t  rbuf_hi_mutex;                    /**< \brief �����ȼ����ͻ������Ƿ�����ʹ�õı�־ */
    uint8_t  rbuf_lo_mutex;                    /**< \brief �����ȼ����ͻ������Ƿ�����ʹ�õı�־ */
    
    uint16_t rx_len;                           /**< \brief �������ݳ��� */
    uint8_t  rx_buf[PRJCONF_UART_RX_BUF_SIZE]; /**< \brief ���ջ�����   */
    
    uint8_t tx_hi_buf[PRJCONF_MODBUS_RESPONSE_HIGH_PRIO_CACHE_SIZE]; /**< \brief �����ȼ����󻺴� */
    uint8_t tx_lo_buf[PRJCONF_MODBUS_RESPONSE_LOW_PRIO_CACHE_SIZE];  /**< \brief �����ȼ����󻺴� */
    uint8_t tx_tmp_buf[PRJCONF_MODBUS_RESPONSE_TMP_CACHE_SIZE];      /**< \brief ��ʱ����         */
};

static struct mb_master_uart_dev m_uart;       /**< \brief �����豸 */
                                               
/** \brief t35��ʱ�� */
APP_TIMER_DEF(mb_uart_t35_timer);

/** \brief ���߾�Ĭ��ʱ�� */
APP_TIMER_DEF(mb_silence_timer);


/** \brief ����������Ӧ��ʱ��� */
static void __mb_uart_response_timeout_check_enable(void)
{
    m_uart.response_timeout_check = true;
    m_uart.response_last_tick = app_timer_cnt_get();
}

/** \brief �رս�����Ӧ��ʱ��� */
static void __mb_uart_response_timeout_check_disable(void)
{
    m_uart.response_timeout_check = false;
}

/** \brief ������Ӧ��ʱ��� */
static bool __mb_uart_response_timeout_poll(void)
{
    m_uart.response_cur_tick = app_timer_cnt_get();
    
    if (app_timer_cnt_diff_compute(m_uart.response_cur_tick, m_uart.response_last_tick) >
        APP_TIMER_TICKS(PRJCONF_MODBUS_RESPONSE_TIMEOUT)) {
        return false;    
    }
    
    return true;
}

/** \brief ����modbus T35��ʱ��� */
static void __mb_uart_t35_enable(void)
{
    m_uart.rx_timeout_check = true;
    m_uart.rx_last_tick = app_timer_cnt_get();
}

/** \brief ֹͣmodbus T35��ʱ��� */
static void __mb_uart_t35_disable(void)
{
    m_uart.rx_timeout_check = false;
}

/** \brief modbus T35��ʱ��ѯ */
static void __mb_uart_t35_timeout_poll(void *p_context)
{
    if (m_uart.rx_timeout_check) {
        m_uart.rx_cur_tick = app_timer_cnt_get();
        if (app_timer_cnt_diff_compute(m_uart.rx_cur_tick, m_uart.rx_last_tick) > APP_TIMER_TICKS(PRJCONF_MODBUS_T35)) {      
            __mb_uart_t35_disable();
            
            app_timer_stop(mb_uart_t35_timer);
            
            if (m_uart.rx_handler)
                m_uart.rx_handler(true, m_uart.rx_buf, m_uart.rx_len);
            
            m_uart.rx_len = 0;
            
            m_uart.state = MB_MASTER_STATE_SILENCE;
            
            app_timer_start(mb_silence_timer, APP_TIMER_TICKS(PRJCONF_MODBUS_SILENCE_TIME), NULL);

#if PRJCONF_DEBUG_ON
            NRF_LOG_INFO("modbus master receive done...");
#endif
        }
    } else {
        if (m_uart.response_timeout_check && !__mb_uart_response_timeout_poll()) {
            if (m_uart.rx_handler) 
                m_uart.rx_handler(false, m_uart.rx_buf, m_uart.rx_len);
            
            __mb_uart_response_timeout_check_disable();
            
            app_timer_stop(mb_uart_t35_timer);
            
            m_uart.state = MB_MASTER_STATE_SILENCE;
            
            app_timer_start(mb_silence_timer, APP_TIMER_TICKS(PRJCONF_MODBUS_SILENCE_TIME), NULL);
            
#if PRJCONF_DEBUG_ON            
            NRF_LOG_INFO("modbus master waiting response timeout...");
#endif            
        }
    }
}

/** \brief modbus�������� */
static void __mb_master_send(uint8_t *tx_buf, uint16_t tx_len)
{
    uint16_t i;
    
    m_uart.state = MB_MASTER_STATE_SENDING;
    
#if PRJCONF_DEBUG_ON    
    NRF_LOG_INFO("modbus sending...");
#endif
	
	m_uart.sid_cache  = tx_buf[0];
	m_uart.fc_cache   = tx_buf[1];
	m_uart.addr_cache = WORD(tx_buf[2], tx_buf[3]);

    app_uart_flush();
    
    PRJCONF_UART_TX_ENABLE;
    
    for (i = 0; i < tx_len; ++i) 
        while (app_uart_put(tx_buf[i]) != NRF_SUCCESS);
}

/** \brief modbus���߾�Ĭ��ʱ����ʱ�ص����� */
static void __mb_silence_expired(void *p_context)
{
    uint32_t nbytes, i = PRJCONF_RINGBUF_MUTEX_CHECK_COUNT;
    
#if PRJCONF_DEBUG_ON
    NRF_LOG_INFO("modbus master silence timer expired.");
#endif    

    if (!ringbuf_isempty(&m_uart.tx_high_prio_buf)) {
        
        while (m_uart.rbuf_hi_mutex && i--);
        m_uart.rbuf_hi_mutex = 1;
        
        ringbuf_get_with_length(&m_uart.tx_high_prio_buf, m_uart.tx_tmp_buf, &nbytes);
        
        m_uart.rbuf_hi_mutex = 0;
        
        __mb_master_send(m_uart.tx_tmp_buf, (uint16_t)nbytes);
        
    } else if (!ringbuf_isempty(&m_uart.tx_low_prio_buf)) {
        
        while (m_uart.rbuf_lo_mutex && i--);
        m_uart.rbuf_lo_mutex = 1;
        
        ringbuf_get_with_length(&m_uart.tx_low_prio_buf, m_uart.tx_tmp_buf, &nbytes);
        
        m_uart.rbuf_lo_mutex = 0;
        
        __mb_master_send(m_uart.tx_tmp_buf, (uint16_t)nbytes);
        
    } else {
        m_uart.state = MB_MASTER_STATE_IDLE;
    }
}

/**
 * \brief Function for handling app_uart events
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
    uint8_t c;
    
    switch (p_event->evt_type) {
        case APP_UART_DATA_READY:
            if (m_uart.state == MB_MASTER_STATE_RECEIVING) {
                app_uart_get(&m_uart.rx_buf[m_uart.rx_len++]);
                __mb_uart_t35_enable();
            } else {
                app_uart_get(&c);
            }
            break;

		case APP_UART_TX_EMPTY:
            PRJCONF_UART_RX_ENABLE;
        
            app_uart_rx_enable();
        
#if PRJCONF_DEBUG_ON            
            NRF_LOG_INFO("modbus master send done...");
#endif        
        
            app_timer_start(mb_uart_t35_timer, APP_TIMER_TICKS(PRJCONF_MODBUS_T35), NULL);
            
            __mb_uart_response_timeout_check_enable();
            
            m_uart.state = MB_MASTER_STATE_RECEIVING;
            
#if PRJCONF_DEBUG_ON            
            NRF_LOG_INFO("modbus master receiving, start tick:%d...", m_uart.response_last_tick);
#endif         
			break;
            
        case APP_UART_FIFO_ERROR:
            NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
            APP_ERROR_HANDLER(p_event->data.error_code);
           break;

        default:
            break;
    }
}

void mb_master_init(void)
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
	
	/* ����485ʹ������ */
	nrf_gpio_cfg_output(PRJCONF_UART_DE);
    
    m_uart.rx_len = 0;
    m_uart.rx_timeout_check = false;
    m_uart.rx_handler = NULL;
    
    m_uart.state = MB_MASTER_STATE_IDLE;
    
    m_uart.rbuf_hi_mutex = 0;
    m_uart.rbuf_lo_mutex = 0;
    
    ringbuf_init(&m_uart.tx_high_prio_buf, m_uart.tx_hi_buf, PRJCONF_MODBUS_RESPONSE_HIGH_PRIO_CACHE_SIZE);
    ringbuf_init(&m_uart.tx_low_prio_buf,  m_uart.tx_lo_buf, PRJCONF_MODBUS_RESPONSE_LOW_PRIO_CACHE_SIZE);
    
	err_code = app_timer_create(&mb_uart_t35_timer, APP_TIMER_MODE_REPEATED, __mb_uart_t35_timeout_poll);
	APP_ERROR_CHECK(err_code);
    
    err_code = app_timer_create(&mb_silence_timer, APP_TIMER_MODE_SINGLE_SHOT, __mb_silence_expired);
	APP_ERROR_CHECK(err_code);
}

void mb_master_request(uint8_t *tx_buf, uint16_t tx_len)
{    
    uint32_t i = PRJCONF_RINGBUF_MUTEX_CHECK_COUNT;
    
    /* ��ǰmodbus������ռ�ã���ǰ���������ݴ����ݣ�����ǰ�����ݴ� */
    if (m_uart.state != MB_MASTER_STATE_IDLE) {
        if (tx_buf[1] == MB_FC_WRITE_COIL  || tx_buf[1] == MB_FC_WRITE_REG ||
            tx_buf[1] == MB_FC_WRITE_COILS || tx_buf[1] == MB_FC_WRITE_REGS) {
                
            while (m_uart.rbuf_hi_mutex && i--);
            m_uart.rbuf_hi_mutex = 1;
                
            if (ringbuf_freebytes(&m_uart.tx_high_prio_buf) > (tx_len + sizeof(uint32_t)))
                ringbuf_put_with_length(&m_uart.tx_high_prio_buf, tx_buf, tx_len);

            m_uart.rbuf_hi_mutex = 0;
            
        } else if (tx_buf[1] == MB_FC_READ_COILS        || tx_buf[1] == MB_FC_READ_INPUT_COILS || 
                   tx_buf[1] == MB_FC_READ_HOLDING_REGS || tx_buf[1] == MB_FC_READ_INPUT_REGS) {
                       
            while (m_uart.rbuf_lo_mutex && i--);
            m_uart.rbuf_lo_mutex = 1;
                       
            if (ringbuf_freebytes(&m_uart.tx_low_prio_buf) > (tx_len + sizeof(uint32_t)))
                ringbuf_put_with_length(&m_uart.tx_low_prio_buf, tx_buf, tx_len);
            
            m_uart.rbuf_lo_mutex = 0;
        }
                   
        return;
    }
    
    __mb_master_send(tx_buf, tx_len);
}

void mb_master_response_handler_register(pfn_recv_data_handler handler)
{
    m_uart.rx_handler = handler;
}

uint8_t mb_master_get_sid_cache(void)
{
    return m_uart.sid_cache;
}

uint8_t mb_master_get_fc_cache(void)
{
    return m_uart.fc_cache;
}

uint16_t mb_master_get_addr_cache(void)
{
    return m_uart.addr_cache;
}
