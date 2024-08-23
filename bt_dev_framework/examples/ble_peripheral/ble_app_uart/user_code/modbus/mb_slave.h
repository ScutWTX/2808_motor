#ifndef __MB_SLAVE_H
 #define __MB_SLAVE_H
 
 #include "mb_netif.h"
 
/** \brief ǰ������,modbus�ӻ����� */
struct mb_slave_serv;

/** \brief modbus�ӻ���� */
typedef struct mb_slave_serv *mb_slave_handle;


/** \brief �������Ӧ������ָ������ */
typedef mb_exception_t (*pfn_mb_fn_code_handler_t)(mb_slave_handle slave, uint8_t *p_frame, uint16_t *p_len);

/** \brief �����봦��ӿ� */
typedef struct mb_fn_code_handle {
    uint8_t                  fn_code; /**< \brief ������                 */
    pfn_mb_fn_code_handler_t handler; /**< \brief �������Ӧ��������� */
} mb_fn_code_handle_t;


/** \brief modbus�ӻ�״̬ */
typedef enum mb_slave_stat {
    MB_STATE_UNUSE = 0, /**< \brief δʹ�� */
    MB_STATE_INIT,      /**< \brief ��ʼ�� */
    MB_STATE_DISABLE,   /**< \brief ����   */
    MB_STATE_ENABLE,    /**< \brief ʹ��   */
} mb_slave_stat_t;

/** \brief modbus�ӻ����� */
typedef struct mb_slave_serv {
    void *p_cookie;                                /**< \brief �ӻ��豸ָ��                 */
     
    const mb_netif_t *p_netif;                     /**< \brief �ӿ�                         */
        
    pfn_mb_fn_code_handler_t *p_handlers;          /**< \brief ͨ�ù������Ӧ�ص�����       */
    
    mb_fn_code_handle_t      *p_customer_handlers; /**< \brief �û��Զ��幦���뼰��Ӧ����   */
    uint8_t max_customer_handlers;                 /**< \brief ���֧���û��Զ��幦�������� */
} mb_slave_serv_t;

/** \brief �ӻ��豸 */
typedef struct mb_slave_dev {
    mb_slave_serv_t  serv;                         /**< \brief modbus�ӻ�����               */
    
    mb_mode_t        mode;                         /**< \brief ����ģʽ                     */
    mb_event_t       event;                        /**< \brief �¼�                         */
    mb_slave_stat_t  stat;                         /**< \brief �ӻ�״̬                     */
    
    union {
        struct mb_rtu rtu;                         /**< \brief RTUģʽ����                  */
    } common;
} mb_slave_dev_t;


/** \brief �ӻ���ʼ�� */
mb_slave_handle mb_slave_init(mb_slave_dev_t *p_dev, uint8_t slave_addr, enum mb_mode mode);

/** \brief �����ӻ� */
mb_err_t mb_slave_start(mb_slave_handle handle);

/** \brief ֹͣ�ӻ� */
mb_err_t mb_slave_stop(mb_slave_handle handle);

/** \brief ע�Ṧ���봦���� */
mb_err_t mb_slave_register_handler(mb_slave_handle handle, uint8_t fncode, mb_fn_code_handle_t fn_code_handler);
 
#endif /* __MB_SLAVE_H */
