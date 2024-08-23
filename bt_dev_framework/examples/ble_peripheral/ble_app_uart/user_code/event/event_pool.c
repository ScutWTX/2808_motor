/**
 * \file
 * \brief event pool, for allocating and freeing
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#include "prjconfig.h"
#include "event_pool.h"

struct event_pool {
    struct list_head free;
    event_t pool[PRJCONF_EVENT_POOL_SIZE];
};

static struct event_pool m_epool;

void event_pool_init(void)
{
    uint16_t i;
    
    INIT_LIST_HEAD(&m_epool.free);
    
    for (i = 0; i < PRJCONF_EVENT_POOL_SIZE; ++i)
        list_add(&m_epool.pool[i].node, &m_epool.free);
}

event_t *event_pool_alloc(void)
{
    if (list_empty(&m_epool.free))
        return NULL;
    
    return list_first_entry(&m_epool.free, event_t, node);
}

void event_pool_free(event_t *e)
{
    list_add_tail(&e->node, &m_epool.free);
}
