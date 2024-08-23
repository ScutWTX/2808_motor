#ifndef __MB_CRC_H
#define __MB_CRC_H

#include <stdint.h>

/** 
 * CRC16����
 *
 * Polynomial: x^16 + x^15 + x^2 + 1 (0xA001)<br>
 * Initial value: 0xFFFF
 *
 * @param[in] frame: ֡���ݻ����� (0x0000..0xFFFF)
 * @param[in] len  : ֡���ݳ��� (0x00..0xFF)
 * @return CRC (0x0000..0xFFFF)
 */
uint16_t mb_crc16(uint8_t *p_frame, uint16_t len);


#endif /* __MB_CRC_H */
