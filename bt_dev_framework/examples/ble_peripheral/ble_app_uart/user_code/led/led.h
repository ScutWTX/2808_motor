#ifndef __LED_H
#define __LED_H

/** 
 * \brief led初始化
 * \note 调用此函数前，必须先初始化用户定时器模块
 */
void led_init(void);

/** \brief led亮 */
void led_on(void);

/** \brief led灭 */
void led_off(void);

/** \brief led闪烁一次 */
void led_blink(void);

#endif /* __LED_H */
