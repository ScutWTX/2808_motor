/**
 * \file
 * \brief event
 *
 * \internal
 * \par modification history
 * - 1.00 2021-04-08  lwx, first implementation
 * \endinternal
 */

#ifndef __EVENT_H
#define __EVENT_H
 
#include "nrf.h"
#include <stdint.h>
#include <stdbool.h>
#include "list.h"

/** \brief 一次调度最大处理时间个数 */
#define CONFIG_EVT_LOOP_ONCE_SCHEDULE_MAX_EVT_COUNT 1

#define READY_SUB_PRIORITY_MASK  0x3F /**< \brief 子优先级掩码 */
#define READY_GRP_PRIORITY_MASK  0xC0 /**< \brief 组优先级掩码 */
#define READY_GRP_PRIORITY_SHIFT 6    /**< \brief 组优先级偏移 */

/** \brief 优先级 */
enum {
    EVT_HIGHEST_GRP_PRIO = 0xC0,      /**< \brief 最高优先级组 */
    EVT_HIGH_GRP_PRIO    = 0x80,      /**< \brief 次高优先级组 */
    EVT_MID_GRP_PRIO     = 0x40,      /**< \brief 中优先级组   */
    EVT_LOWER_GRP_PRIO   = 0x00,      /**< \brief 低优先级组   */
    READY_GRP_COUNT  = 4              /**< \brief 优先级组个数 */
};

struct event;                                  /**< \brief 前向声明         */
typedef struct event event_t;                  /**< \brief 事件类型         */
typedef void (*event_cb_t)(void *, event_t *); /**< \brief 事件回调函数类型 */

/** \brief 事件结构体 */
struct event {
    struct list_head node;   /**< \brief 节点         */                                      
    void *p_context;         /**< \brief 事件上下文   */
    event_cb_t pfn_callback; /**< \brief 事件回调函数 */
    bool    is_ready;        /**< \brief 是否就绪     */
    uint8_t priority;        /**< \brief 事件优先级   */
};

/** \brief 构造事件优先级
 * \param grp_prio: 组优先级(EVT_HIGHEST_GRP_PRIO, EVT_HIGH_GRP_PRIO, EVT_MID_GRP_PRIO, EVT_LOWER_GRP_PRIO)
 * \param sub_prio: 组内子优先级(优先级从低到高:0~63)
 */
#define EVT_MAKE_PRIORITY(grp_prio, sub_prio) \
    (((grp_prio) & READY_GRP_PRIORITY_MASK) | ((sub_prio) & READY_SUB_PRIORITY_MASK))

/** \brief 事件初始化 */ 
void event_init(event_t *p_evt, event_cb_t cb, void *p_ctx, uint8_t prio);

/** \brief 事件继承初始化 */
void event_init_inherit(event_t *p_evt, const event_t *p_parent_evt);


/** \brief 事件循环结构体 */
typedef struct event_loop {
    struct list_head ready_grps[READY_GRP_COUNT]; /**< \brief 事件就绪队列组       */
    uint8_t ready_map;                            /**< \brief 事件组就绪状态bitmap */
} event_loop_t;

/** \brief 事件循环初始化 */
void event_loop_init(event_loop_t *el);

/** \brief 向事件循环提交事件 */
bool event_loop_event_post(event_loop_t *el, event_t *e);

/** \brief 取消事件循环中的一个事件 */
bool event_loop_event_cancel(event_loop_t *el, event_t *e);

/** \brief 事件优先级重置 */
bool event_loop_event_priority_reset(event_loop_t *el, event_t *e, uint8_t new_prio);

/** \brief 事件调度 */
void event_loop_schedule(event_loop_t *el);

/************************************************************************************/

/** \brief 检查事件类型是否合法 */
#define PENDING_EVENT_TYPE_ALLOC_CHECK(event_type) (((event_type) >= PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS) ? false : true)

/** \brief 事件 */
typedef struct {
    uint8_t   event_type; /**< \brief 事件类型 */
	uint16_t  length;     /**< \brief 数据长度 */
	uint8_t  *p_data;     /**< \brief 数据指针 */
} pending_event_t;

/** \brief 事件处理函数类型，p_data为pending_event_t中的p_data, length为pending_event_t中的length */
typedef void (*pfn_pending_event_handler)(uint8_t *p_data, uint16_t length);

/** \brief 事件处理模块初始化 */
void pending_event_module_init(void);

/** \brief 事件类型分配，由用户自行存储，用于pending_event_handler_register接口 */
uint8_t pending_event_type_alloc(void);

/** \brief 事件处理函数注册 */
bool pending_event_handler_register(uint8_t event_type, pfn_pending_event_handler handler);

/** \brief 事件上报 */
uint32_t pending_event_report(pending_event_t *evt);

/** \brief 事件处理，需要在循环体内调用 */
void pending_event_schedule(void);

#endif /* __EVENT_H */
