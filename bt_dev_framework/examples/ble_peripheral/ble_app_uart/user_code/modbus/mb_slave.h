#ifndef __MB_SLAVE_H
 #define __MB_SLAVE_H
 
 #include "mb_netif.h"
 
/** \brief 前向声明,modbus从机服务 */
struct mb_slave_serv;

/** \brief modbus从机句柄 */
typedef struct mb_slave_serv *mb_slave_handle;


/** \brief 功能码对应处理函数指针类型 */
typedef mb_exception_t (*pfn_mb_fn_code_handler_t)(mb_slave_handle slave, uint8_t *p_frame, uint16_t *p_len);

/** \brief 功能码处理接口 */
typedef struct mb_fn_code_handle {
    uint8_t                  fn_code; /**< \brief 功能码                 */
    pfn_mb_fn_code_handler_t handler; /**< \brief 功能码对应处理函数句柄 */
} mb_fn_code_handle_t;


/** \brief modbus从机状态 */
typedef enum mb_slave_stat {
    MB_STATE_UNUSE = 0, /**< \brief 未使用 */
    MB_STATE_INIT,      /**< \brief 初始化 */
    MB_STATE_DISABLE,   /**< \brief 禁能   */
    MB_STATE_ENABLE,    /**< \brief 使能   */
} mb_slave_stat_t;

/** \brief modbus从机服务 */
typedef struct mb_slave_serv {
    void *p_cookie;                                /**< \brief 从机设备指针                 */
     
    const mb_netif_t *p_netif;                     /**< \brief 接口                         */
        
    pfn_mb_fn_code_handler_t *p_handlers;          /**< \brief 通用功能码对应回调函数       */
    
    mb_fn_code_handle_t      *p_customer_handlers; /**< \brief 用户自定义功能码及对应处理   */
    uint8_t max_customer_handlers;                 /**< \brief 最大支持用户自定义功能码数量 */
} mb_slave_serv_t;

/** \brief 从机设备 */
typedef struct mb_slave_dev {
    mb_slave_serv_t  serv;                         /**< \brief modbus从机服务               */
    
    mb_mode_t        mode;                         /**< \brief 工作模式                     */
    mb_event_t       event;                        /**< \brief 事件                         */
    mb_slave_stat_t  stat;                         /**< \brief 从机状态                     */
    
    union {
        struct mb_rtu rtu;                         /**< \brief RTU模式数据                  */
    } common;
} mb_slave_dev_t;


/** \brief 从机初始化 */
mb_slave_handle mb_slave_init(mb_slave_dev_t *p_dev, uint8_t slave_addr, enum mb_mode mode);

/** \brief 启动从机 */
mb_err_t mb_slave_start(mb_slave_handle handle);

/** \brief 停止从机 */
mb_err_t mb_slave_stop(mb_slave_handle handle);

/** \brief 注册功能码处理函数 */
mb_err_t mb_slave_register_handler(mb_slave_handle handle, uint8_t fncode, mb_fn_code_handle_t fn_code_handler);
 
#endif /* __MB_SLAVE_H */
