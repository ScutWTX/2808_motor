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

/** \brief һ�ε��������ʱ����� */
#define CONFIG_EVT_LOOP_ONCE_SCHEDULE_MAX_EVT_COUNT 1

#define READY_SUB_PRIORITY_MASK  0x3F /**< \brief �����ȼ����� */
#define READY_GRP_PRIORITY_MASK  0xC0 /**< \brief �����ȼ����� */
#define READY_GRP_PRIORITY_SHIFT 6    /**< \brief �����ȼ�ƫ�� */

/** \brief ���ȼ� */
enum {
    EVT_HIGHEST_GRP_PRIO = 0xC0,      /**< \brief ������ȼ��� */
    EVT_HIGH_GRP_PRIO    = 0x80,      /**< \brief �θ����ȼ��� */
    EVT_MID_GRP_PRIO     = 0x40,      /**< \brief �����ȼ���   */
    EVT_LOWER_GRP_PRIO   = 0x00,      /**< \brief �����ȼ���   */
    READY_GRP_COUNT  = 4              /**< \brief ���ȼ������ */
};

struct event;                                  /**< \brief ǰ������         */
typedef struct event event_t;                  /**< \brief �¼�����         */
typedef void (*event_cb_t)(void *, event_t *); /**< \brief �¼��ص��������� */

/** \brief �¼��ṹ�� */
struct event {
    struct list_head node;   /**< \brief �ڵ�         */                                      
    void *p_context;         /**< \brief �¼�������   */
    event_cb_t pfn_callback; /**< \brief �¼��ص����� */
    bool    is_ready;        /**< \brief �Ƿ����     */
    uint8_t priority;        /**< \brief �¼����ȼ�   */
};

/** \brief �����¼����ȼ�
 * \param grp_prio: �����ȼ�(EVT_HIGHEST_GRP_PRIO, EVT_HIGH_GRP_PRIO, EVT_MID_GRP_PRIO, EVT_LOWER_GRP_PRIO)
 * \param sub_prio: ���������ȼ�(���ȼ��ӵ͵���:0~63)
 */
#define EVT_MAKE_PRIORITY(grp_prio, sub_prio) \
    (((grp_prio) & READY_GRP_PRIORITY_MASK) | ((sub_prio) & READY_SUB_PRIORITY_MASK))

/** \brief �¼���ʼ�� */ 
void event_init(event_t *p_evt, event_cb_t cb, void *p_ctx, uint8_t prio);

/** \brief �¼��̳г�ʼ�� */
void event_init_inherit(event_t *p_evt, const event_t *p_parent_evt);


/** \brief �¼�ѭ���ṹ�� */
typedef struct event_loop {
    struct list_head ready_grps[READY_GRP_COUNT]; /**< \brief �¼�����������       */
    uint8_t ready_map;                            /**< \brief �¼������״̬bitmap */
} event_loop_t;

/** \brief �¼�ѭ����ʼ�� */
void event_loop_init(event_loop_t *el);

/** \brief ���¼�ѭ���ύ�¼� */
bool event_loop_event_post(event_loop_t *el, event_t *e);

/** \brief ȡ���¼�ѭ���е�һ���¼� */
bool event_loop_event_cancel(event_loop_t *el, event_t *e);

/** \brief �¼����ȼ����� */
bool event_loop_event_priority_reset(event_loop_t *el, event_t *e, uint8_t new_prio);

/** \brief �¼����� */
void event_loop_schedule(event_loop_t *el);

/************************************************************************************/

/** \brief ����¼������Ƿ�Ϸ� */
#define PENDING_EVENT_TYPE_ALLOC_CHECK(event_type) (((event_type) >= PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS) ? false : true)

/** \brief �¼� */
typedef struct {
    uint8_t   event_type; /**< \brief �¼����� */
	uint16_t  length;     /**< \brief ���ݳ��� */
	uint8_t  *p_data;     /**< \brief ����ָ�� */
} pending_event_t;

/** \brief �¼����������ͣ�p_dataΪpending_event_t�е�p_data, lengthΪpending_event_t�е�length */
typedef void (*pfn_pending_event_handler)(uint8_t *p_data, uint16_t length);

/** \brief �¼�����ģ���ʼ�� */
void pending_event_module_init(void);

/** \brief �¼����ͷ��䣬���û����д洢������pending_event_handler_register�ӿ� */
uint8_t pending_event_type_alloc(void);

/** \brief �¼�������ע�� */
bool pending_event_handler_register(uint8_t event_type, pfn_pending_event_handler handler);

/** \brief �¼��ϱ� */
uint32_t pending_event_report(pending_event_t *evt);

/** \brief �¼�������Ҫ��ѭ�����ڵ��� */
void pending_event_schedule(void);

#endif /* __EVENT_H */
