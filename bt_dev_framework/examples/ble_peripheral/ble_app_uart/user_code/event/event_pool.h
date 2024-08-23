/**
 * \file
 * \brief event pool header file
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#ifndef __EVENT_POOL_H
#define __EVENT_POOL_H
 
#include "event.h"
 
struct event_pool;
typedef struct event_pool event_pool_t;

/** \brief 事件池初始化 */
void event_pool_init(void);

/** \brief 获取一个可使用的事件 */
event_t *event_pool_alloc(void);

/** \brief 释放一个不再使用的事件 */
void event_pool_free(event_t *e);

#endif /* __EVENT_POOL_H */
