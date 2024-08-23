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

/** \brief ���յ����ݵĴ���״̬ */
enum mb_pdu_proc_state {
    MB_STATE_PDU_NO_PROCESS, /**< \brief ���ڴ�����  */
    MB_STATE_PDU_IN_PROCESS, /**< \brief ���ڴ���    */
};

/** \brief RTU����״̬ */
enum mb_rtu_recv_state {
    MB_RTU_STATE_RX_INIT,  /**< \brief ��ʼ�� */
    MB_RTU_STATE_RX_IDLE,  /**< \brief ����   */
    MB_RTU_STATE_RX_RECV,  /**< \brief ����   */
    MB_RTU_STATE_RX_ERROR, /**< \brief ����   */
};

/** \brief RTU����״̬ */
enum mb_rtu_send_state {
    MB_RTU_STATE_TX_IDLE, /**< \brief ���� */
    MB_RTU_STATE_TX_XMIT, /**< \brief ���� */
};

/** \brief RTU˽������ */
struct mb_rtu {
    enum mb_rtu_recv_state rx_state;  /**< \brief ����״̬ */
    enum mb_rtu_send_state tx_state;  /**< \brief ����״̬ */
    
#if MB_SER_PDU_RECV_BUF_ENABLED > 0
    enum mb_pdu_proc_state pdu_state;  /**< \brief PDU���ݽ���״̬ */
    uint8_t pdu_buf_proc_pos;          /**< \brief PDU buffer����λ�� */
#endif
    
    uint8_t slave_addr;   /**< \brief �ӻ���ַ        */
    uint8_t pdu_buf_pos;  /**< \brief ����PDU����λ�� */
    uint16_t tx_data_len; /**< \brief �������ݳ���    */
    uint8_t *tx_data;     /**< \brief ��������buffer  */
    
    /** \brief PDU buffer */
    uint8_t pdu_buf[__MB_SER_ADU_BUF_MAX][MB_MAX_SER_ADU_LENGTH];
    /** \brief �������ݼ��� */
    uint16_t rx_data_pos[__MB_SER_ADU_BUF_MAX];
    
    /** \brief ��վ�¼� */
    mb_event_t event;
};

/** \brief ������·ͨ�ýӿ� */
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
