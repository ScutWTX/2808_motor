#ifndef __MB_SLAVE_UART_H
#define __MB_SLAVE_UART_H

#include <stdint.h>

/** \brief ����uartͨѶ��modbus�ӻ���ʼ�� */
void mb_slave_uart_init(void);

/** \brief ��uart���յ������ݽ��д��� */
void mb_slave_uart_rxdata_handling(uint8_t const *p_data, uint16_t len);

#endif /* __MB_SLAVE_UART_H */
