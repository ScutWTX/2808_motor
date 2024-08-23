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

/** \brief �¼��س�ʼ�� */
void event_pool_init(void);

/** \brief ��ȡһ����ʹ�õ��¼� */
event_t *event_pool_alloc(void);

/** \brief �ͷ�һ������ʹ�õ��¼� */
void event_pool_free(event_t *e);

#endif /* __EVENT_POOL_H */
