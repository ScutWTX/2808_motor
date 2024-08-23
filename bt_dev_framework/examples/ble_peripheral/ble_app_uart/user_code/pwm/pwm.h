#ifndef __PWM_H 
#define __PWM_H

#include <stdint.h>
#include "app_pwm.h"

/** \brief PWM双通道设备 */
typedef struct pwm_dev {
    uint32_t freq_hz;     /**< \brief PWM频率 */
    uint32_t pwm_a_pin;   /**< \brief PWM_A输出引脚 */
    uint32_t pwm_b_pin;   /**< \brief PWM_B输出引脚 */
    
    const app_pwm_t *pwm_handle; /**< \brief PWM设备 */
} pwm_dev_t, *pwm_handle_t;

/** \brief PWM初始化 */
pwm_handle_t pwm_init(uint32_t freq_hz, uint32_t pwm_a_pin, uint32_t pwm_b_pin);

/** \brief 停止输出 */
void pwm_output_stop(pwm_handle_t handle);

/** 
 * \brief PWM_A输出PWM信号，PWM_B低电平输出
 * \param[in] handle: PWM双通道设备
 * \param[in] duty_x10: 占空比，分辨率0.1%
 */
void pwm_output_cw(pwm_handle_t handle, uint16_t duty_x10);

/** 
 * \brief PWM_A低电平输出，PWM_B输出PWM信号
 * \param[in] handle: PWM双通道设备
 * \param[in] duty_x10: 占空比，分辨率0.1%
 */
void pwm_output_ccw(pwm_handle_t handle, uint16_t duty_x10);


#endif /* __PWM_H */
