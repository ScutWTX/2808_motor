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
 * \brief Modbus数据打包
 *
 * \param[in]  sid:         从机地址
 * \param[in]  fc:          功能码
 * \param[in]  addr:        寄存器或者线圈地址
 * \param[in]  p_user_data: 用户传入数据,写线圈或者写寄存器时的数据指针
 * \param[in]  qty:         用户数据数量,写多个线圈时为线圈个数,写多个寄存器时为寄存器个数
 * \param[out] p_pdu:       Modbus帧数据指针
 * \param[out] p_len:       Modbus帧数据长度指针
 *
 * \return     void
 */
void mb_master_pdu_packing(uint8_t sid, uint8_t fc, uint16_t addr, 
                           uint16_t *p_user_data, uint16_t qty, 
                           uint8_t *p_pdu, volatile uint16_t *p_len);


/**
 * \brief modbus解包并将数据部分存放在uint16_t缓冲区
 */
mb_exception_t mb_master_pdu_unpacking(uint8_t *p_buf, uint16_t len, uint16_t *p_data, uint16_t data_max_len);

#endif /* __MB_MASTER_H */
