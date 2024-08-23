#ifndef __DRV8432_H
#define __DRV8432_H

#include "pwm.h"
#include "encoder.h"

/** \brief DRV8432设备 */
typedef struct drv8432_dev {
    uint32_t reset_ab_pin;           /**< \brief 复位AB引脚       */
    uint32_t reset_cd_pin;           /**< \brief 复位CD引脚       */
    uint32_t fault_pin;              /**< \brief 错误信号引脚     */
    uint32_t otw_pin;                /**< \brief 温度过高信号引脚 */
    
    pwm_handle_t     pwm_handle;     /**< \brief PWM设备句柄      */
    encoder_handle_t encoder_handle; /**< \brief 编码器设备引脚   */
} drv8432_dev_t, *drv8432_handle_t;

/** \brief DRV8432初始化 */
drv8432_handle_t drv8432_init(void);

/** \brief DRV8432复位 */
void drv8432_reset(drv8432_handle_t handle);

/** \brief 当前DRV8432是否为错误状态 */
bool drv8432_fault_stat(drv8432_handle_t handle);

/** \brief 当前DRV8432是否为过热状态 */
bool drv8432_otw_stat(drv8432_handle_t handle);

/** \brief 当前编码器反馈线速度 */
uint16_t drv8432_get_encoder_wire_speed(drv8432_handle_t handle);

#endif /* __DRV8432_H */
