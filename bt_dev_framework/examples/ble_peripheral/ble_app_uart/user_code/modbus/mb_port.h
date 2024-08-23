/**
 * \file
 * \brief modbus ported header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#include "mb_common.h"

/** \brief modbus�¼� */
typedef enum mb_event {
    MB_EVENT_INIT,     /**< \brief �¼���ʼ��    */
    MB_EVENT_READY,    /**< \brief �����¼�      */
    MB_EVENT_RECEIVED, /**< \brief ���յ�1֡���� */
    MB_EVENT_SENDING,  /**< \brief ���ڷ���      */
    MB_EVENT_SEND_OUT, /**< \brief ���ݷ������  */
    MB_EVENT_STOP,     /**< \brief ֹͣ�¼�      */
    MB_EVENT_TIMEOUT,  /**< \brief ��ʱ�¼�      */
} mb_event_t;

/** \brief �¼���ʼ�� */
mb_err_t mb_port_event_init(void);

/** \brief �����¼� */
mb_err_t mb_port_event_post(mb_event_t e);

/** \brief ��ѯ�¼� */
mb_err_t mb_port_event_get(mb_event_t *pe);
