#ifndef __PWM_H 
#define __PWM_H

#include <stdint.h>
#include "app_pwm.h"

/** \brief PWM˫ͨ���豸 */
typedef struct pwm_dev {
    uint32_t freq_hz;     /**< \brief PWMƵ�� */
    uint32_t pwm_a_pin;   /**< \brief PWM_A������� */
    uint32_t pwm_b_pin;   /**< \brief PWM_B������� */
    
    const app_pwm_t *pwm_handle; /**< \brief PWM�豸 */
} pwm_dev_t, *pwm_handle_t;

/** \brief PWM��ʼ�� */
pwm_handle_t pwm_init(uint32_t freq_hz, uint32_t pwm_a_pin, uint32_t pwm_b_pin);

/** \brief ֹͣ��� */
void pwm_output_stop(pwm_handle_t handle);

/** 
 * \brief PWM_A���PWM�źţ�PWM_B�͵�ƽ���
 * \param[in] handle: PWM˫ͨ���豸
 * \param[in] duty_x10: ռ�ձȣ��ֱ���0.1%
 */
void pwm_output_cw(pwm_handle_t handle, uint16_t duty_x10);

/** 
 * \brief PWM_A�͵�ƽ�����PWM_B���PWM�ź�
 * \param[in] handle: PWM˫ͨ���豸
 * \param[in] duty_x10: ռ�ձȣ��ֱ���0.1%
 */
void pwm_output_ccw(pwm_handle_t handle, uint16_t duty_x10);


#endif /* __PWM_H */
