/**
 * \file
 * \brief event
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */
 
#include "prjconfig.h"

#include "event.h"
#include "nrf_queue.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void event_init(event_t *p_evt, event_cb_t cb, void *p_ctx, uint8_t prio)
{
    INIT_LIST_HEAD(&p_evt->node);
    p_evt->pfn_callback  = cb ? cb : NULL;
    p_evt->p_context     = p_ctx;
    p_evt->priority      = prio;
    p_evt->is_ready      = false;
}

void event_init_inherit(event_t *p_evt, const event_t *p_parent_evt)
{
    INIT_LIST_HEAD(&p_evt->node);
    p_evt->pfn_callback  = p_parent_evt->pfn_callback;
    p_evt->p_context     = p_parent_evt->p_context;
    p_evt->priority      = p_parent_evt->priority;
    p_evt->is_ready      = false;
}

static inline
void __event_fifo_priority_push(struct list_head *p_fifo, event_t *e)
{
    event_t *tmp, *cur;
    
    struct list_head *pre;
    
    cur = list_last_entry(p_fifo, event_t, node);
    
    if (list_empty(p_fifo) || e->priority <= cur->priority) {
        list_add_tail(&e->node, p_fifo);
    } else {
        pre = p_fifo;
        
        list_for_each_entry_safe(cur, tmp, p_fifo, event_t, node) {
            if (e->priority > cur->priority)
                break;
            
            pre = &cur->node;
        }
        
        list_add(&e->node, (pre == p_fifo) ? p_fifo : pre);
    }
}

static inline
event_t *__event_fifo_priority_pop(struct list_head *p_fifo)
{
    event_t *e;
    
    if (list_empty(p_fifo))
        return NULL;
    
    e = list_first_entry(p_fifo, event_t, node);
    
    list_del_init(&e->node);
    
    return e;
}

static inline
bool __event_fifo_priority_reset(struct list_head *p_fifo, event_t *e, uint8_t new_prio)
{
    event_t *tmp, *cur;
    
    struct list_head *pre, *ins_pos;
    
    uint8_t is_find_prio = 0, is_find_event = 0;
    
    if (list_last_entry(p_fifo, event_t, node) == e && new_prio <= e->priority) {
        e->priority = new_prio;
        return true;
    }
    
    pre     = p_fifo;
    ins_pos = pre;
    
    list_for_each_entry_safe(cur, tmp, p_fifo, event_t, node) {
        if (!is_find_prio && new_prio > cur->priority) {
            is_find_prio = 1;
            ins_pos = pre;
        }
        
        if (cur == e) {
            list_del(&cur->node);
            is_find_event = 1;
        }
        
        if (is_find_event && is_find_prio)
            break;
        
        pre = &cur->node;
    }
    
    if (!is_find_event)
        return false;
    
    e->priority = new_prio;
    list_add(&e->node, ins_pos);
    
    return true;
}

void event_loop_init(event_loop_t *el)
{
    int i;
    
    if (!el)
        return;
    
    for (i = 0; i < READY_GRP_COUNT; ++i) {
        INIT_LIST_HEAD(&el->ready_grps[i]);
    }
    
    el->ready_map = 0;
}

static inline 
uint8_t __event_loop_private_highest_rdy_grp_get(event_loop_t *el)
{
    static const uint8_t priority_rdy_bitmap[1 << READY_GRP_COUNT] =  {
        0xFF, 0, 1, 1, 2, 2, 2, 2, /* 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111 */
           3, 3, 3, 3, 3, 3, 3, 3, /* 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111 */
    };
    
    return priority_rdy_bitmap[el->ready_map];
}

static inline
uint8_t __event_loop_imm_event(event_loop_t *el)
{
    return el->ready_map;
}

bool event_loop_event_post(event_loop_t *el, event_t *e)
{
    uint8_t rdy_grp = e->priority >> READY_GRP_PRIORITY_SHIFT;
    
    if (list_empty(&e->node)) {
        __event_fifo_priority_push(&el->ready_grps[rdy_grp], e);
        
        e->is_ready = true;
        
        el->ready_map |= (1 << rdy_grp);
        
        return true;
    }
    
    return false;
}

bool event_loop_event_cancel(event_loop_t *el, event_t *e)
{
    uint8_t rdy_grp = e->priority >> READY_GRP_PRIORITY_SHIFT;
    
    if (rdy_grp >= READY_GRP_COUNT)
        return false;
    
    if (!list_empty(&e->node)) {
        list_del_init(&e->node);
        
        if (list_empty(&el->ready_grps[rdy_grp]))
            el->ready_map &= ~(1 << rdy_grp);
        
        e->is_ready = false;
        
        return true;
    }
    
    return false;
}

bool event_loop_event_priority_reset(event_loop_t *el, event_t *e, uint8_t new_prio)
{
    uint8_t cur_grp, new_grp;
    struct list_head *cur_rdy_q, *new_rdy_q;
    
    cur_grp = e->priority >> READY_GRP_PRIORITY_SHIFT;
    new_grp = new_prio    >> READY_GRP_PRIORITY_SHIFT;
    
    cur_rdy_q = &el->ready_grps[cur_grp];
    new_rdy_q = &el->ready_grps[new_grp];
    
    if (list_empty(&e->node))
        return false;
    
    if (cur_grp == new_grp) {
        __event_fifo_priority_reset(cur_rdy_q, e, new_prio);
    } else {
        list_del_init(&e->node);
        
        if (list_empty(cur_rdy_q))
            el->ready_map &= ~(1 << cur_grp);
        
        e->priority = new_prio;
        __event_fifo_priority_push(new_rdy_q, e);
        el->ready_map |= 1 << new_grp;
    }
    
    return true;
}

static inline
void __event_loop_private_event_schedule(event_loop_t *el)
{
    event_t *e;
    
    uint8_t rdy_grp = __event_loop_private_highest_rdy_grp_get(el);
    
    if (rdy_grp < READY_GRP_COUNT) {
        e = __event_fifo_priority_pop(&el->ready_grps[rdy_grp]);
        
        if (list_empty(&el->ready_grps[rdy_grp]))
            el->ready_map &= ~(1 << rdy_grp);
        
        e->pfn_callback(e->p_context, e);
        
        e->is_ready = false;
    }
}

void event_loop_schedule(event_loop_t *el)
{
    int max_schedule_events = CONFIG_EVT_LOOP_ONCE_SCHEDULE_MAX_EVT_COUNT;
    
    while (__event_loop_imm_event(el) && max_schedule_events--)
        __event_loop_private_event_schedule(el);
}

/******************************************************************************/

NRF_QUEUE_DEF(pending_event_t, pending_event_queue, 50 * sizeof(pending_event_t), NRF_QUEUE_MODE_OVERFLOW);

/** \brief 事件管理模块 */
typedef struct __pending_event_module {
    pfn_pending_event_handler handlers[PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS];
    uint64_t maps;
} pending_event_module;

static pending_event_module m_pending_event_module;

void pending_event_module_init(void)
{
    int i = 0;
    
    m_pending_event_module.maps = 0;
    
    for (i = 0; i < PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS; ++i) 
        m_pending_event_module.handlers[i] = NULL;
}

uint8_t pending_event_type_alloc(void)
{
    uint8_t i;
    
    for (i = 0; i < PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS; ++i) {
        if (!(m_pending_event_module.maps & (1 << i))) {
            m_pending_event_module.maps |= (1 << i);
            break;
        }
    }
    
    return i;
}

bool pending_event_handler_register(uint8_t event_type, pfn_pending_event_handler handler)
{
    if (event_type >= PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS || NULL == handler)
        return false;
    
    m_pending_event_module.handlers[event_type] = handler;
    
    return true;
}


uint32_t pending_event_report(pending_event_t *evt)
{
	return nrf_queue_push(&pending_event_queue, evt);
}

void pending_event_schedule(void)
{
    ret_code_t ret;
	pending_event_t event_info;

	ret = nrf_queue_pop(&pending_event_queue, &event_info);
    
    if (ret == NRF_SUCCESS) {
        if (event_info.event_type >= PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS)
            return;
        
        if (m_pending_event_module.handlers[event_info.event_type]) {
            m_pending_event_module.handlers[event_info.event_type](event_info.p_data, event_info.length);
        } else {
            if (m_pending_event_module.maps | (1 << event_info.event_type)) {
                NRF_LOG_INFO("PENDING_EVENT: event type is allocated, but event type handler is not register.");
            } else {
                NRF_LOG_INFO("PENDING_EVENT: event type is not allocated.");
            }
        }
    }
}
