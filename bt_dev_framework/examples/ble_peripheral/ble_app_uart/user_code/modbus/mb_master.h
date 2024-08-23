/**
 * \file
 * \brief modbus master function
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-13  lwx, first implementation
 * \endinternal
 */
 
#ifndef __MB_MASTER_H
#define __MB_MASTER_H

#include <stdint.h>
#include "mb_common.h"

/**
 * \brief Modbus���ݴ��
 *
 * \param[in]  sid:         �ӻ���ַ
 * \param[in]  fc:          ������
 * \param[in]  addr:        �Ĵ���������Ȧ��ַ
 * \param[in]  p_user_data: �û���������,д��Ȧ����д�Ĵ���ʱ������ָ��
 * \param[in]  qty:         �û���������,д�����ȦʱΪ��Ȧ����,д����Ĵ���ʱΪ�Ĵ�������
 * \param[out] p_pdu:       Modbus֡����ָ��
 * \param[out] p_len:       Modbus֡���ݳ���ָ��
 *
 * \return     void
 */
void mb_master_pdu_packing(uint8_t sid, uint8_t fc, uint16_t addr, 
                           uint16_t *p_user_data, uint16_t qty, 
                           uint8_t *p_pdu, volatile uint16_t *p_len);


/**
 * \brief modbus����������ݲ��ִ����uint16_t������
 */
mb_exception_t mb_master_pdu_unpacking(uint8_t *p_buf, uint16_t len, uint16_t *p_data, uint16_t data_max_len);

#endif /* __MB_MASTER_H */
