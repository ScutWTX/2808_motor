#ifndef __DRV8432_H
#define __DRV8432_H

#include "pwm.h"
#include "encoder.h"

/** \brief DRV8432�豸 */
typedef struct drv8432_dev {
    uint32_t reset_ab_pin;           /**< \brief ��λAB����       */
    uint32_t reset_cd_pin;           /**< \brief ��λCD����       */
    uint32_t fault_pin;              /**< \brief �����ź�����     */
    uint32_t otw_pin;                /**< \brief �¶ȹ����ź����� */
    
    pwm_handle_t     pwm_handle;     /**< \brief PWM�豸���      */
    encoder_handle_t encoder_handle; /**< \brief �������豸����   */
} drv8432_dev_t, *drv8432_handle_t;

/** \brief DRV8432��ʼ�� */
drv8432_handle_t drv8432_init(void);

/** \brief DRV8432��λ */
void drv8432_reset(drv8432_handle_t handle);

/** \brief ��ǰDRV8432�Ƿ�Ϊ����״̬ */
bool drv8432_fault_stat(drv8432_handle_t handle);

/** \brief ��ǰDRV8432�Ƿ�Ϊ����״̬ */
bool drv8432_otw_stat(drv8432_handle_t handle);

/** \brief ��ǰ�������������ٶ� */
uint16_t drv8432_get_encoder_wire_speed(drv8432_handle_t handle);

#endif /* __DRV8432_H */
