/**
 * \file
 * \brief uart header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */

#ifndef __MB_MASTER_UART_H 
#define __MB_MASTER_UART_H

#include <stdint.h>
#include <stdbool.h>

/** \brief 接收数据处理函数指针类型 */
typedef void (*pfn_recv_data_handler)(bool rx_status, uint8_t *pbuf, uint16_t plen);

/**\brief modbus主机初始化 */
void mb_master_init(void);

/** \brief 主机发送请求 */
void mb_master_request(uint8_t *tx_buf, uint16_t tx_len);

/** \brief 注册接收数据处理函数 */
void mb_master_response_handler_register(pfn_recv_data_handler handler);

/** \brief 获取当前请求时的从机地址缓存 */
uint8_t mb_master_get_sid_cache(void);

/** \brief 获取当前请求时的功能码缓存 */
uint8_t mb_master_get_fc_cache(void);

/** \brief 获取当前请求时的地址缓存 */
uint16_t mb_master_get_addr_cache(void);

#endif /* __MB_MASTER_UART_H */
