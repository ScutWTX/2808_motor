#ifndef __UART_INIT
#define __UART_INIT
#include <stdint.h>
#include <stdbool.h>


#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                        /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */
#define PRJ_CONF_DEV_NAME "FWD_3078"

/** \brief ���ڽ������� */
#define PRJ_CONF_UART_RX_PIN NRF_GPIO_PIN_MAP(0, 25)
/** \brief ���ڷ������� */
#define PRJ_CONF_UART_TX_PIN NRF_GPIO_PIN_MAP(0, 23)
/** \brief �������ݷ���������ţ��ߵ�ƽ����ʹ�ܣ��͵�ƽ����ʹ�� */
#define PRJ_CONF_UART_DE     NRF_GPIO_PIN_MAP(0, 24)
/** \brief ���ڽ���ʹ�� */
#define PRJ_CONF_UART_RX_ENABLE nrf_gpio_pin_clear(PRJ_CONF_UART_DE)
/** \brief ���ڷ���ʹ�� */
#define PRJ_CONF_UART_TX_ENABLE nrf_gpio_pin_set(PRJ_CONF_UART_DE)


extern bool ack_received_flag;
extern bool ack_waiting_flag;



extern uint8_t rdata_buf[UART_RX_BUF_SIZE];

void timers_init(void);
void timeout_handler(void *p_context);

void UART_Init_9600(void);
void UART_Init_115200(void);
void UART_WriteData(uint8_t *pData, uint8_t dataLen);
void init_gpio(void);
void rdata_handling(uint8_t* data,uint16_t length);

static void timer_handler(void *p_context);

#endif
