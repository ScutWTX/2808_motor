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

#define MB_PDU_SIZE_MAX 253  /**< \brief 最大PDU长度         */
#define MB_PDU_SIZE_MIN 1    /**< \brief 最小PDU长度         */
#define MB_PDU_FUNC_OFF 0    /**< \brief PDU中功能码位置偏移 */
#define MB_PDU_DATA_OFF 1    /**< \brief PDU中数据位置偏移   */

/** \brief modbus工作模式 */
 typedef enum mb_mode {
     MB_MODE_RTU,   /**< \brief RTU传输模式   */
     MB_MODE_ASCII, /**< \brief ASCII传输模式 */
     MB_MODE_TCP,   /**< \brief TCP模式       */
 } mb_mode_t;

/** \brief modbus功能码 */
typedef enum mb_func_code {
    MB_FC_READ_COILS        = 0x01, /**< \brief 读取多个输出线圈状态 */
    MB_FC_READ_INPUT_COILS  = 0x02, /**< \brief 读取多个输入线圈状态 */
    MB_FC_READ_HOLDING_REGS = 0x03, /**< \brief 读取多个保持寄存器   */
    MB_FC_READ_INPUT_REGS   = 0x04, /**< \brief 读取多个输入寄存器   */
    MB_FC_WRITE_COIL        = 0x05, /**< \brief 写入单个线圈状态     */
    MB_FC_WRITE_REG         = 0x06, /**< \brief 写入单个寄存器       */
    MB_FC_WRITE_COILS       = 0x0F, /**< \brief 写入多个线圈状态     */
    MB_FC_WRITE_REGS        = 0x10, /**< \brief 写入多个寄存器       */
    MB_FC_ERROR             = 0x80, /**< \brief 异常功能码           */
} mb_func_code_t;

/** \brief modbus异常码 */
typedef enum mb_exception {
    MB_EXC_NONE                 = 0x00, /**< \brief 无异常               */
    MB_EXC_ILLEGAL_FUNCTION     = 0x01, /**< \brief 非法功能码           */
    MB_EXC_ILLEGAL_DATA_ADDRESS = 0x02, /**< \brief 非法数据地址         */
    MB_EXC_ILLEGAL_DATA_VALUE   = 0x03, /**< \brief 非法数据值           */
    MB_EXC_SLAVE_DEVICE_FAILURE = 0x04, /**< \brief 从站设备故障         */
    MB_EXC_ACKNOWLEDGE          = 0x05, /**< \brief 确认                 */
    MB_EXC_SLAVE_DEVICE_BUSY    = 0x06, /**< \brief 从站设备忙           */  
    MB_EXC_MEMORY_PARITY_ERROR  = 0x08, /**< \brief 存储奇偶性错         */
    MB_EXC_GATEWAY_PATH_FAILED  = 0x0A, /**< \brief 不可用网关路径       */
    MB_EXC_GATEWAY_TGT_FAILED   = 0x0B, /**< \brief 网关目标设备响应失败 */
    
    MB_EXC_CRC_ERROR,                   /**< \brief CRC错误              */
    MB_EXC_DATA_BUF_OVERFLOW,           /**< \brief 缓存数据溢出风险     */
    MB_EXC_INVAL_PARAM,                 /**< \brief 非法参数              */
} mb_exception_t;

/** \brief 错误码 */
typedef enum mb_err {
    MB_ENONE,     /**< \brief 无错误         */
    MB_ENOREG,    /**< \brief 非法寄存器地址 */
    MB_EINVAL,    /**< \brief 非法参数       */
    MB_EPROTERR,  /**< \brief 移植层错误     */
    MB_ENORES,    /**< \brief 资源不足       */
    MB_EIO,       /**< \brief IO错误         */
    MB_EILLSTATE, /**< \brief 协议栈非法状态 */
    MB_ETIMEOUT   /**< \brief 超时错误       */
} mb_err_t;

/** \brief 获取低8位数据 */
#define LOW_BYTE(ww)  ((uint8_t)((ww) & 0x00FF))
/** \brief 获取高8位数据 */
#define HIGH_BYTE(ww) ((uint8_t)((ww) >> 8))
/** \brief 两个8位数据组合成16位数据 */
#define WORD(hb, lb)  ((uint16_t)(((uint8_t)(hb)) << 8) | ((uint8_t)(lb)))

#endif /* __MB_COMMON_H */
