/**
 * \file
 * \brief modbus master function
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-13  lwx, first implementation
 * \endinternal
 */

#include "mb_master.h"
#include "mb_crc.h"


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
                           uint8_t *p_pdu, volatile uint16_t *p_len)
{
    uint8_t  i, _qty;
    uint16_t crc;
    uint16_t  adu_size = 0;
	
	// assemble Modbus Request Application Data Unit
    p_pdu[adu_size++] = sid;
    p_pdu[adu_size++] = fc;
	
	switch (fc) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_INPUT_COILS:
        case MB_FC_READ_HOLDING_REGS:
        case MB_FC_READ_INPUT_REGS:
            p_pdu[adu_size++] = HIGH_BYTE(addr);
            p_pdu[adu_size++] = LOW_BYTE(addr);
            p_pdu[adu_size++] = HIGH_BYTE(qty);
            p_pdu[adu_size++] = LOW_BYTE(qty);
            break;
        default:
            break;
    }
	
    switch (fc) {
        case MB_FC_WRITE_COIL:
        case MB_FC_WRITE_REG:
        case MB_FC_WRITE_COILS:
        case MB_FC_WRITE_REGS:
            p_pdu[adu_size++] = HIGH_BYTE(addr);
            p_pdu[adu_size++] = LOW_BYTE(addr);
            break;
        default:
            break;
    }
	
	switch (fc) {
        case MB_FC_WRITE_COIL:
            p_pdu[adu_size++] = HIGH_BYTE(qty);
            p_pdu[adu_size++] = LOW_BYTE(qty);
            break;
        
        case MB_FC_WRITE_REG:
            p_pdu[adu_size++] = HIGH_BYTE(p_user_data[0]);
            p_pdu[adu_size++] = LOW_BYTE(p_user_data[0]);
            break;
      
        case MB_FC_WRITE_COILS:
            p_pdu[adu_size++] = HIGH_BYTE(qty);
            p_pdu[adu_size++] = LOW_BYTE(qty);
            _qty = (qty % 8) ? ((qty >> 3) + 1) : (qty >> 3);
            p_pdu[adu_size++] = _qty;
            for (i = 0; i < _qty; i++) {
                switch (i % 2) {
                    case 0: //i is even
                        p_pdu[adu_size++] = LOW_BYTE(p_user_data[i >> 1]);
                        break;
                    case 1: //i is odd
                        p_pdu[adu_size++] = HIGH_BYTE(p_user_data[i >> 1]);
                        break;
                }
            }
            break;
      
        case MB_FC_WRITE_REGS:
            p_pdu[adu_size++] = HIGH_BYTE(qty);
            p_pdu[adu_size++] = LOW_BYTE(qty);
            p_pdu[adu_size++] = LOW_BYTE(qty << 1);
            
            for (i = 0; i < LOW_BYTE(qty); i++) {
                p_pdu[adu_size++] = HIGH_BYTE(p_user_data[i]);
                p_pdu[adu_size++] = LOW_BYTE(p_user_data[i]);
            }
            break;
        
        default:
            break;
    }
	
    crc = mb_crc16(p_pdu, adu_size);
    
    p_pdu[adu_size++] = LOW_BYTE(crc);
    p_pdu[adu_size++] = HIGH_BYTE(crc);
    *p_len = adu_size;
}

/**
 * \brief modbus解包并将数据部分存放在uint16_t缓冲区
 */
mb_exception_t mb_master_pdu_unpacking(uint8_t *p_buf, uint16_t len, uint16_t *p_data, uint16_t data_max_len)
{
    int i;
    
    uint16_t crc = mb_crc16(p_buf, len - 2);
    
    if (crc != WORD(p_buf[len - 1], p_buf[len - 2])) 
        return MB_EXC_CRC_ERROR;
    
    //Exception check
    if (p_buf[1] & 0x80)
        return (mb_exception_t)p_buf[2];
    
    switch (p_buf[1]) {
        case MB_FC_READ_COILS:
        case MB_FC_READ_INPUT_COILS:
            if ((p_buf[2] >> 1) > data_max_len)
                return MB_EXC_DATA_BUF_OVERFLOW;                
        
            for (i = 0; i < (p_buf[2] >> 1); ++i)
                p_data[i] = WORD(p_buf[2 * i + 4], p_buf[2 * i + 3]);
            
            if (p_buf[2] & 0x1)
                p_data[i] = WORD(0, p_buf[2 * i + 3]);
            break;
            
        case MB_FC_READ_HOLDING_REGS:
        case MB_FC_READ_INPUT_REGS:
            if ((p_buf[2] >> 1) > data_max_len)
                return MB_EXC_DATA_BUF_OVERFLOW;     
        
            for (i = 0; i < (p_buf[2] >> 1); ++i)
                p_data[i] = WORD(p_buf[2 * i + 3], p_buf[2 * i + 4]);
            break;
            
        default:
            break;
    }
    
    return MB_EXC_NONE;
}
