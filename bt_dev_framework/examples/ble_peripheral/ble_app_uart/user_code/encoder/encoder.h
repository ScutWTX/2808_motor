#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include "nrf_drv_ppi.h"

enum encoder_dir {
    ENCODER_DIR_NONE = 0, /**< \brief 无方向，无输入状态 */
    ENCODER_DIR_CW,       /**< \brief 正转               */
    ENCODER_DIR_CCW       /**< \brief 反转               */
};

/** \brief 编码器设备 */
typedef struct encoder_dev {
    nrf_ppi_channel_t channel_a;        /**< \brief PPI通道a */
    nrf_ppi_channel_t channel_b;        /**< \brief PPI通道b */
                                        
    uint32_t pin_enc_a;                 /**< \brief 编码器输出引脚A */
    uint32_t pin_enc_b;                 /**< \brief 编码器输出引脚B */
                                        
    volatile uint32_t freq_hz;          /**< \brief 计算得到的频率，分辨率1hz */
    volatile uint32_t rpm;              /**< \brief 转每分钟，分辨率0.1rpm    */
    volatile uint32_t wire_speed;       /**< \brief 线速度，分辨率:0.1mm/min  */
                                        
    volatile enum encoder_dir dir;      /**< \brief 判断得到的方向 */
    
    volatile uint32_t last_capture_cnt; /**< \brief 上一个捕获的定时器计数值 */
    volatile uint32_t cur_capture_cnt;  /**< \brief 当前捕获的定时器计数值   */
} encoder_dev_t, *encoder_handle_t;


/** \brief 测速编码器初始化 */
encoder_handle_t encoder_init(uint32_t enc_a_pin, uint32_t enc_b_pin);

/** \brief 获取方向判断值 */
enum encoder_dir encoder_get_dir(encoder_handle_t handle); 

/** \brief 获取编码器信号输出频率，分辨率Hz */
uint32_t encoder_get_freq(encoder_handle_t handle);

/** \brief 获取转速，分辨率0.1rpm */
uint32_t encoder_get_rpm(encoder_handle_t handle);

/** \brief 获取转速，分辨率1mm/min */
uint16_t encoder_get_wire_speed(encoder_handle_t handle);

#endif /* __ENCODER_H */
