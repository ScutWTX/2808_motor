/**
 * \file
 * \brief modbus common header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#ifndef __MB_COMMON_H
#define __MB_COMMON_H

#include <stdint.h>

#define MB_PDU_SIZE_MAX 253  /**< \brief ���PDU����         */
#define MB_PDU_SIZE_MIN 1    /**< \brief ��СPDU����         */
#define MB_PDU_FUNC_OFF 0    /**< \brief PDU�й�����λ��ƫ�� */
#define MB_PDU_DATA_OFF 1    /**< \brief PDU������λ��ƫ��   */

/** \brief modbus����ģʽ */
 typedef enum mb_mode {
     MB_MODE_RTU,   /**< \brief RTU����ģʽ   */
     MB_MODE_ASCII, /**< \brief ASCII����ģʽ */
     MB_MODE_TCP,   /**< \brief TCPģʽ       */
 } mb_mode_t;

/** \brief modbus������ */
typedef enum mb_func_code {
    MB_FC_READ_COILS        = 0x01, /**< \brief ��ȡ��������Ȧ״̬ */
    MB_FC_READ_INPUT_COILS  = 0x02, /**< \brief ��ȡ���������Ȧ״̬ */
    MB_FC_READ_HOLDING_REGS = 0x03, /**< \brief ��ȡ������ּĴ���   */
    MB_FC_READ_INPUT_REGS   = 0x04, /**< \brief ��ȡ�������Ĵ���   */
    MB_FC_WRITE_COIL        = 0x05, /**< \brief д�뵥����Ȧ״̬     */
    MB_FC_WRITE_REG         = 0x06, /**< \brief д�뵥���Ĵ���       */
    MB_FC_WRITE_COILS       = 0x0F, /**< \brief д������Ȧ״̬     */
    MB_FC_WRITE_REGS        = 0x10, /**< \brief д�����Ĵ���       */
    MB_FC_ERROR             = 0x80, /**< \brief �쳣������           */
} mb_func_code_t;

/** \brief modbus�쳣�� */
typedef enum mb_exception {
    MB_EXC_NONE                 = 0x00, /**< \brief ���쳣               */
    MB_EXC_ILLEGAL_FUNCTION     = 0x01, /**< \brief �Ƿ�������           */
    MB_EXC_ILLEGAL_DATA_ADDRESS = 0x02, /**< \brief �Ƿ����ݵ�ַ         */
    MB_EXC_ILLEGAL_DATA_VALUE   = 0x03, /**< \brief �Ƿ�����ֵ           */
    MB_EXC_SLAVE_DEVICE_FAILURE = 0x04, /**< \brief ��վ�豸����         */
    MB_EXC_ACKNOWLEDGE          = 0x05, /**< \brief ȷ��                 */
    MB_EXC_SLAVE_DEVICE_BUSY    = 0x06, /**< \brief ��վ�豸æ           */  
    MB_EXC_MEMORY_PARITY_ERROR  = 0x08, /**< \brief �洢��ż�Դ�         */
    MB_EXC_GATEWAY_PATH_FAILED  = 0x0A, /**< \brief ����������·��       */
    MB_EXC_GATEWAY_TGT_FAILED   = 0x0B, /**< \brief ����Ŀ���豸��Ӧʧ�� */
    
    MB_EXC_CRC_ERROR,                   /**< \brief CRC����              */
    MB_EXC_DATA_BUF_OVERFLOW,           /**< \brief ���������������     */
    MB_EXC_INVAL_PARAM,                 /**< \brief �Ƿ�����              */
} mb_exception_t;

/** \brief ������ */
typedef enum mb_err {
    MB_ENONE,     /**< \brief �޴���         */
    MB_ENOREG,    /**< \brief �Ƿ��Ĵ�����ַ */
    MB_EINVAL,    /**< \brief �Ƿ�����       */
    MB_EPROTERR,  /**< \brief ��ֲ�����     */
    MB_ENORES,    /**< \brief ��Դ����       */
    MB_EIO,       /**< \brief IO����         */
    MB_EILLSTATE, /**< \brief Э��ջ�Ƿ�״̬ */
    MB_ETIMEOUT   /**< \brief ��ʱ����       */
} mb_err_t;

/** \brief ��ȡ��8λ���� */
#define LOW_BYTE(ww)  ((uint8_t)((ww) & 0x00FF))
/** \brief ��ȡ��8λ���� */
#define HIGH_BYTE(ww) ((uint8_t)((ww) >> 8))
/** \brief ����8λ������ϳ�16λ���� */
#define WORD(hb, lb)  ((uint16_t)(((uint8_t)(hb)) << 8) | ((uint8_t)(lb)))

#endif /* __MB_COMMON_H */
