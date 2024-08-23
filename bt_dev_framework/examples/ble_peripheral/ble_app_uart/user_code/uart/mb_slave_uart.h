#ifndef __MB_SLAVE_UART_H
#define __MB_SLAVE_UART_H

#include <stdint.h>

/** \brief 基于uart通讯的modbus从机初始化 */
void mb_slave_uart_init(void);

/** \brief 将uart接收到的数据进行处理 */
void mb_slave_uart_rxdata_handling(uint8_t const *p_data, uint16_t len);

#endif /* __MB_SLAVE_UART_H */
