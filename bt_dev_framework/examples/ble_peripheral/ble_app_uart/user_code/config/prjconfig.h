#ifndef __PRJCONFIG_H
#define __PRJCONFIG_H

#include <stdbool.h>
#include "nrf_gpio.h"
#include "nrf_uart.h"

/** \brief ������Ϣ��ӡ���أ�0���ܣ���0ʹ�� */
#define PRJCONF_DEBUG_ON (0)

/******************************************************************************
 ************************** communicate interface******************************
 *****************************************************************************/
 #define COMMUNICATE_INTERFACE_RS485 (0)
 #define COMMUNICATE_INTERFACE_BLE   (1)
 #define PRJCONF_COMMUNICATE_INTERFACE COMMUNICATE_INTERFACE_RS485
 

/******************************************************************************
 ************************** wire feed motor ***********************************
 *****************************************************************************/
#define WFM_TYPE_GNL_32_21E   0 /**< \brief reduction ratio:1/32, encoder wires:120 */
#define WFM_TYPE_FWF_840_0    1 /**< \brief reduction ratio:1/24, encoder wires:480(���ٵ��) */
#define WFM_TYPE_FWF_840_1    2 /**< \brief reduction ratio:1/50, encoder wires:120(���ٵ��) */    

/** \brief ��ǰ��˿������� */
#define PRJCONF_CURRENT_WFM_TYPE WFM_TYPE_FWF_840_1


/******************************************************************************
 ************************** bluetooth config **********************************
 *****************************************************************************/
 
/** \brief ����longrangeģʽ���ã�0���ܣ���0ʹ�� */
#define PRJCONF_BT_MODE_LONGRANGE (1)

/** 
 * \brief �����豸���� 
 * FWD_1xxx: �ŷ����ư��豸
 * FWD_2xxx: ң�����豸
 * FWD_3xxx: ��˿���豸
 * FWD_4xxx: ��λ���豸
 */
#define PRJCONF_BT_DEV_NAME "FWD_3078" /**< Name of device. Will be included in the advertising data. */

/******************************************************************************
 **************************** wirefeeder config *******************************
 *****************************************************************************/
enum { 
    WFR_MOTOR_CW = 0, /**< \brief ��˿�������ת */
    WFR_MOTOR_CCW     /**< \brief ��˿�������ת */
};

#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_GNL_32_21E

/** \brief ��˿����˿������ */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153��x:��˿�ٶ�(mm/min)��y:PWMռ�ձ�(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief ��˿������ٶ�6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_0

/** \brief ��˿����˿������ */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CW

/** \brief y = 0.0398x+15.121��x:��˿�ٶ�(mm/min)��y:PWMռ�ձ�(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (598)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (15)

/** \brief ��˿������ٶ�21000mm/min */
#define PRJCONF_WFR_MAX_SPEED 21000
#define PRJCONF_WFR_MIN_SPEED 0

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1

/** \brief ��˿����˿������ */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153��x:��˿�ٶ�(mm/min)��y:PWMռ�ձ�(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief ��˿������ٶ�6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#else

/** \brief ��˿����˿������ */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153��x:��˿�ٶ�(mm/min)��y:PWMռ�ձ�(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief ��˿������ٶ�6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#endif


 
/******************************************************************************
 **************************** event config ************************************
 *****************************************************************************/

/** \brief �¼��ش�С */
#define PRJCONF_EVENT_POOL_SIZE (1)

/** \brief ���֧���¼����������� */
#define PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS      (16)

#define PRJCONF_EVENT_MAX_SUPPORT_FUNCTION_SLOTS  (64)

#ifdef PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS
    #if PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS > PRJCONF_EVENT_MAX_SUPPORT_FUNCTION_SLOTS
        #error "PRJCONF_EVENT_SUPPORT_FUNCTION_SLOTS must less than or equal to 64"
    #endif
#endif


/******************************************************************************
 ************************* communication LED config ***************************
 *****************************************************************************/
 
/** \brief LED��Ӧ���� */
#define PRJCONF_COMM_LED_PIN NRF_GPIO_PIN_MAP(0, 11)

/** \brief LEDϨ��ʱ�䣬��λ:ms */
#define PRJCONF_COMM_LED_LIGHTOFF_TIME (50)

/** \brief ͨ�ŵ��� */
#define PRJCONF_COMM_LED_ON  nrf_gpio_pin_set(PRJCONF_COMM_LED_PIN)

/** \brief ͨ�ŵ��� */
#define PRJCONF_COMM_LED_OFF nrf_gpio_pin_clear(PRJCONF_COMM_LED_PIN)


/******************************************************************************
 **************************** modbus config ***********************************
 *****************************************************************************/
 
#define PRJCONF_UART_TX_BUF_SIZE (256) /**< \brief UART TX buffer size. */
#define PRJCONF_UART_RX_BUF_SIZE (256) /**< \brief UART RX buffer size. */

/** \brief ���ڲ����� */
#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_115200
//#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_38400
//#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_9600

/**
 * \brief ����У�鷽ʽ
 * \note When parity is enabled through the PARITY field in the CONFIG register, the parity will be generated
 *       automatically from the even parity of TXD and RXD for transmission and reception respectively.
 *       The amount of stop bits can be configured through the STOP field in the CONFIG register.
 *       ��֧����У�飬��֧��żУ�顢��У�飡��
 */
enum {
    __UART_PARITY_NONE = false, /**< \brief ��У�� */
    __UART_PARITY_EVEN = true   /**< \brief żУ�� */
};

#define PRJCONF_UART_PARITY __UART_PARITY_EVEN

/** \brief ���ڽ������� */
#define PRJCONF_UART_RX_PIN NRF_GPIO_PIN_MAP(0, 3)
/** \brief ���ڷ������� */
#define PRJCONF_UART_TX_PIN NRF_GPIO_PIN_MAP(0, 28)
/** \brief �������ݷ���������ţ��ߵ�ƽ����ʹ�ܣ��͵�ƽ����ʹ�� */
#define PRJCONF_UART_DE     NRF_GPIO_PIN_MAP(0, 2)

/** \brief ���ڽ���ʹ�� */
#define PRJCONF_UART_RX_ENABLE nrf_gpio_pin_clear(PRJCONF_UART_DE)

/** \brief ���ڷ���ʹ�� */
#define PRJCONF_UART_TX_ENABLE nrf_gpio_pin_set(PRJCONF_UART_DE)

/** \brief modbus֡��T35��ʱʱ�䣬�����ʴ���19200ʱ��T35=1750us��������С�ڵ���19200ʱ��T35=3.5*10/baudrate */
#define PRJCONF_MODBUS_T35 (2) //< ��λ:ms


/******************************************************************************
 **************************** DRV8432 config **********************************
 *****************************************************************************/

/** \brief ���������A��doc:WFMControl.pdf  */
#define PRJCONF_DRV8432_ENC_A_PIN   NRF_GPIO_PIN_MAP(0, 15)

/** \brief ���������B */
#define PRJCONF_DRV8432_ENC_B_PIN   NRF_GPIO_PIN_MAP(0, 14)

/** \brief PWM_A��������*/
#define PRJCONF_DRV8432_PWMA_PIN    NRF_GPIO_PIN_MAP(0, 19)

/** \brief PWM_B�������� */
#define PRJCONF_DRV8432_PWMB_PIN    NRF_GPIO_PIN_MAP(0, 21)

/** \brief RESET_AB���� */
#define PRJCONF_DRV8432_RESETAB_PIN NRF_GPIO_PIN_MAP(0, 20)

/** \brief RESET_CD���� */
#define PRJCONF_DRV8432_RESETCD_PIN NRF_GPIO_PIN_MAP(0, 22)

/** \brief OTW(����,����Ч)���� */
#define PRJCONF_DRV8432_OTW_PIN     NRF_GPIO_PIN_MAP(0, 17)

/** \brief FAULT(�����ź�,����Ч)���� */
#define PRJCONF_DRV8432_FAULT_PIN   NRF_GPIO_PIN_MAP(0, 16)

/** \brief PWM���Ƶ�� */
#define PRJCONF_DRV8432_PWM_FREQ                (40000L)

/** \brief ��λ����ʱ�䣬��λ:ms */
#define PRJCONF_DRV8432_RESET_HOLDING_TIME      (500)

/** \brief ������������ʱ������,��λms */
#define PRJCONF_DRV8432_ENCODER_SAMPLE_TIME     (50)

#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_GNL_32_21E

/** \brief ���ٱ� */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (32)

/** \brief �������ֱ���CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_0

/** \brief ���ٱ� */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (24)

/** \brief �������ֱ���CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (480)


#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1

/** \brief ���ٱ� */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (50)

/** \brief �������ֱ���CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#else /* default config */

/** \brief ���ٱ� */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (32)

/** \brief �������ֱ���CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#endif

/** \brief ֱ�� */
#define PRJCONF_DRV8432_MOTOR_DIAMETER          (40)

/******************************************************************************
 **************************** modbus slave config ************************
 *****************************************************************************/
#define PRJCONF_MODBUS_SLAVE_BLE_CACHE_SIZE (512)  /**< \brief MODBUS�ӻ�ble�������ݻ����С */
#define PRJCONF_MODBUS_SLAVE_UART_CACHE_SIZE (512) /**< \brief MODBUS�ӻ�uart�������ݻ����С */

/******************************************************************************
 **************************** event config ************************************
 *****************************************************************************/


#endif /* __PRJCONFIG_H */
