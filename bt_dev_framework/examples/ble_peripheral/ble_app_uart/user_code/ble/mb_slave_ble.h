#ifndef __MB_SLAVE_BLE_H
#define __MB_SLAVE_BLE_H

#include <stdint.h>

/** \brief ����bleͨѶ��modbus�ӻ���ʼ�� */
void mb_slave_ble_init(void);

/** \brief ��ble���յ������ݽ��д��� */
void mb_slave_ble_rxdata_handling(uint8_t const *p_data, uint16_t len);


#endif /* __MB_SLAVE_BLE_H */
