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

/** \brief �������ݴ�����ָ������ */
typedef void (*pfn_recv_data_handler)(bool rx_status, uint8_t *pbuf, uint16_t plen);

/**\brief modbus������ʼ�� */
void mb_master_init(void);

/** \brief ������������ */
void mb_master_request(uint8_t *tx_buf, uint16_t tx_len);

/** \brief ע��������ݴ����� */
void mb_master_response_handler_register(pfn_recv_data_handler handler);

/** \brief ��ȡ��ǰ����ʱ�Ĵӻ���ַ���� */
uint8_t mb_master_get_sid_cache(void);

/** \brief ��ȡ��ǰ����ʱ�Ĺ����뻺�� */
uint8_t mb_master_get_fc_cache(void);

/** \brief ��ȡ��ǰ����ʱ�ĵ�ַ���� */
uint16_t mb_master_get_addr_cache(void);

#endif /* __MB_MASTER_UART_H */
