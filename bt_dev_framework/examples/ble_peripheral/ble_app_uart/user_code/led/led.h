#ifndef __LED_H
#define __LED_H

/** 
 * \brief led��ʼ��
 * \note ���ô˺���ǰ�������ȳ�ʼ���û���ʱ��ģ��
 */
void led_init(void);

/** \brief led�� */
void led_on(void);

/** \brief led�� */
void led_off(void);

/** \brief led��˸һ�� */
void led_blink(void);

#endif /* __LED_H */
