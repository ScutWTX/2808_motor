#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "nrf_drv_ppi.h"

enum encoder_dir {
    ENCODER_DIR_NONE = 0, /**< \brief �޷���������״̬ */
    ENCODER_DIR_CW,       /**< \brief ��ת               */
    ENCODER_DIR_CCW       /**< \brief ��ת               */
};

/** \brief �������豸 */
typedef struct encoder_dev {
    nrf_ppi_channel_t channel_a;        /**< \brief PPIͨ��a */
    nrf_ppi_channel_t channel_b;        /**< \brief PPIͨ��b */
                                        
    uint32_t pin_enc_a;                 /**< \brief �������������A */
    uint32_t pin_enc_b;                 /**< \brief �������������B */
                                        
    volatile uint32_t freq_hz;          /**< \brief ����õ���Ƶ�ʣ��ֱ���1hz */
    volatile uint32_t rpm;              /**< \brief תÿ���ӣ��ֱ���0.1rpm    */
    volatile uint32_t wire_speed;       /**< \brief ���ٶȣ��ֱ���:0.1mm/min  */
                                        
    volatile enum encoder_dir dir;      /**< \brief �жϵõ��ķ��� */
    
    volatile uint32_t last_capture_cnt; /**< \brief ��һ������Ķ�ʱ������ֵ */
    volatile uint32_t cur_capture_cnt;  /**< \brief ��ǰ����Ķ�ʱ������ֵ   */
} encoder_dev_t, *encoder_handle_t;


/** \brief ���ٱ�������ʼ�� */
encoder_handle_t encoder_init(uint32_t enc_a_pin, uint32_t enc_b_pin);

/** \brief ��ȡ�����ж�ֵ */
enum encoder_dir encoder_get_dir(encoder_handle_t handle); 

/** \brief ��ȡ�������ź����Ƶ�ʣ��ֱ���Hz */
uint32_t encoder_get_freq(encoder_handle_t handle);

/** \brief ��ȡת�٣��ֱ���0.1rpm */
uint32_t encoder_get_rpm(encoder_handle_t handle);

/** \brief ��ȡת�٣��ֱ���1mm/min */
uint16_t encoder_get_wire_speed(encoder_handle_t handle);

#endif /* __ENCODER_H */
