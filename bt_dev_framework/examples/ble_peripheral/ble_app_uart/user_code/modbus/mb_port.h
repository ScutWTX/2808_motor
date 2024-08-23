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

/** \brief modbus事件 */
typedef enum mb_event {
    MB_EVENT_INIT,     /**< \brief 事件初始化    */
    MB_EVENT_READY,    /**< \brief 就绪事件      */
    MB_EVENT_RECEIVED, /**< \brief 接收到1帧数据 */
    MB_EVENT_SENDING,  /**< \brief 正在发送      */
    MB_EVENT_SEND_OUT, /**< \brief 数据发送完成  */
    MB_EVENT_STOP,     /**< \brief 停止事件      */
    MB_EVENT_TIMEOUT,  /**< \brief 超时事件      */
} mb_event_t;

/** \brief 事件初始化 */
mb_err_t mb_port_event_init(void);

/** \brief 发送事件 */
mb_err_t mb_port_event_post(mb_event_t e);

/** \brief 查询事件 */
mb_err_t mb_port_event_get(mb_event_t *pe);
