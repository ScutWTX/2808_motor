/**
 * \file
 * \brief modbus net interface
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#ifndef __MB_NETIF_H
#define __MB_NETIF_H

#include "mb_config.h"
#include "mb_port.h"

#if MB_SER_PDU_RECV_BUF_ENABLED
#define __MB_SER_ADU_BUF_MAX 2
#else
#define __MB_SER_ADU_BUF_MAX 1
#endif

/** \brief 接收到数据的处理状态 */
enum mb_pdu_proc_state {
    MB_STATE_PDU_NO_PROCESS, /**< \brief 不在处理中  */
    MB_STATE_PDU_IN_PROCESS, /**< \brief 正在处理    */
};

/** \brief RTU接收状态 */
enum mb_rtu_recv_state {
    MB_RTU_STATE_RX_INIT,  /**< \brief 初始化 */
    MB_RTU_STATE_RX_IDLE,  /**< \brief 空闲   */
    MB_RTU_STATE_RX_RECV,  /**< \brief 接收   */
    MB_RTU_STATE_RX_ERROR, /**< \brief 错误   */
};

/** \brief RTU发送状态 */
enum mb_rtu_send_state {
    MB_RTU_STATE_TX_IDLE, /**< \brief 空闲 */
    MB_RTU_STATE_TX_XMIT, /**< \brief 发送 */
};

/** \brief RTU私有数据 */
struct mb_rtu {
    enum mb_rtu_recv_state rx_state;  /**< \brief 接收状态 */
    enum mb_rtu_send_state tx_state;  /**< \brief 发送状态 */
    
#if MB_SER_PDU_RECV_BUF_ENABLED > 0
    enum mb_pdu_proc_state pdu_state;  /**< \brief PDU数据接收状态 */
    uint8_t pdu_buf_proc_pos;          /**< \brief PDU buffer处理位置 */
#endif
    
    uint8_t slave_addr;   /**< \brief 从机地址        */
    uint8_t pdu_buf_pos;  /**< \brief 接收PDU数据位置 */
    uint16_t tx_data_len; /**< \brief 发送数据长度    */
    uint8_t *tx_data;     /**< \brief 发送数据buffer  */
    
    /** \brief PDU buffer */
    uint8_t pdu_buf[__MB_SER_ADU_BUF_MAX][MB_MAX_SER_ADU_LENGTH];
    /** \brief 接收数据计数 */
    uint16_t rx_data_pos[__MB_SER_ADU_BUF_MAX];
    
    /** \brief 从站事件 */
    mb_event_t event;
};

/** \brief 数据链路通用接口 */
typedef struct mb_netif {
    uint8_t  (*pfn_slave_addr_get)(void *p_cookie);
    mb_err_t (*pfn_slave_addr_set)(void *p_cookie);
    mb_err_t (*pfn_init) (void *p_cookie, const void *p_param, mb_event_t e);
    void     (*pfn_close)(void *p_cookie);
    void     (*pfn_start)(void *p_cookie);
    void     (*pfn_stop) (void *p_cookie);
    mb_err_t (*pfn_recv) (void *p_cookie, uint8_t *p_addr, uint8_t **pp_pdu, uint16_t *p_len);
    mb_err_t (*pfn_send) (void *p_cookie, uint8_t *p_pdu, uint16_t len);
} mb_netif_t;


#endif /* __MB_NETIF_H */
