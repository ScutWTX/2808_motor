#ifndef __PRJCONFIG_H
#define __PRJCONFIG_H

#include <stdbool.h>
#include "nrf_gpio.h"
#include "nrf_uart.h"

/** \brief 调试信息打印开关，0禁能，非0使能 */
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
#define WFM_TYPE_FWF_840_0    1 /**< \brief reduction ratio:1/24, encoder wires:480(高速电机) */
#define WFM_TYPE_FWF_840_1    2 /**< \brief reduction ratio:1/50, encoder wires:120(低速电机) */    

/** \brief 当前送丝电机类型 */
#define PRJCONF_CURRENT_WFM_TYPE WFM_TYPE_FWF_840_1


/******************************************************************************
 ************************** bluetooth config **********************************
 *****************************************************************************/
 
/** \brief 蓝牙longrange模式设置，0禁能，非0使能 */
#define PRJCONF_BT_MODE_LONGRANGE (1)

/** 
 * \brief 蓝牙设备名称 
 * FWD_1xxx: 伺服控制板设备
 * FWD_2xxx: 遥控器设备
 * FWD_3xxx: 送丝机设备
 * FWD_4xxx: 变位机设备
 */
#define PRJCONF_BT_DEV_NAME "FWD_3078" /**< Name of device. Will be included in the advertising data. */

/******************************************************************************
 **************************** wirefeeder config *******************************
 *****************************************************************************/
enum { 
    WFR_MOTOR_CW = 0, /**< \brief 送丝机电机正转 */
    WFR_MOTOR_CCW     /**< \brief 送丝机电机反转 */
};

#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_GNL_32_21E

/** \brief 送丝机进丝方向极性 */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153，x:送丝速度(mm/min)，y:PWM占空比(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief 送丝机最大速度6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_0

/** \brief 送丝机进丝方向极性 */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CW

/** \brief y = 0.0398x+15.121，x:送丝速度(mm/min)，y:PWM占空比(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (598)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (15)

/** \brief 送丝机最大速度21000mm/min */
#define PRJCONF_WFR_MAX_SPEED 21000
#define PRJCONF_WFR_MIN_SPEED 0

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1

/** \brief 送丝机进丝方向极性 */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153，x:送丝速度(mm/min)，y:PWM占空比(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief 送丝机最大速度6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#else

/** \brief 送丝机进丝方向极性 */
#define PRJCONF_WFR_POLARITY_OF_PUSH WFR_MOTOR_CCW

/** \brief y = 0.1285x+18.153，x:送丝速度(mm/min)，y:PWM占空比(0.1%) */
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_A (1285)
#define PRJCONF_WFR_SPEED_2_PWM_LINEAR_B (10)

/** \brief 送丝机最大速度6500mm/min */
#define PRJCONF_WFR_MAX_SPEED 6500
#define PRJCONF_WFR_MIN_SPEED 0

#endif


 
/******************************************************************************
 **************************** event config ************************************
 *****************************************************************************/

/** \brief 事件池大小 */
#define PRJCONF_EVENT_POOL_SIZE (1)

/** \brief 最大支持事件处理函数个数 */
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
 
/** \brief LED对应引脚 */
#define PRJCONF_COMM_LED_PIN NRF_GPIO_PIN_MAP(0, 11)

/** \brief LED熄灭时间，单位:ms */
#define PRJCONF_COMM_LED_LIGHTOFF_TIME (50)

/** \brief 通信灯亮 */
#define PRJCONF_COMM_LED_ON  nrf_gpio_pin_set(PRJCONF_COMM_LED_PIN)

/** \brief 通信灯灭 */
#define PRJCONF_COMM_LED_OFF nrf_gpio_pin_clear(PRJCONF_COMM_LED_PIN)


/******************************************************************************
 **************************** modbus config ***********************************
 *****************************************************************************/
 
#define PRJCONF_UART_TX_BUF_SIZE (256) /**< \brief UART TX buffer size. */
#define PRJCONF_UART_RX_BUF_SIZE (256) /**< \brief UART RX buffer size. */

/** \brief 串口波特率 */
#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_115200
//#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_38400
//#define PRJCONF_UART_BAUDRATE NRF_UART_BAUDRATE_9600

/**
 * \brief 串口校验方式
 * \note When parity is enabled through the PARITY field in the CONFIG register, the parity will be generated
 *       automatically from the even parity of TXD and RXD for transmission and reception respectively.
 *       The amount of stop bits can be configured through the STOP field in the CONFIG register.
 *       不支持奇校验，仅支持偶校验、无校验！！
 */
enum {
    __UART_PARITY_NONE = false, /**< \brief 无校验 */
    __UART_PARITY_EVEN = true   /**< \brief 偶校验 */
};

#define PRJCONF_UART_PARITY __UART_PARITY_EVEN

/** \brief 串口接收引脚 */
#define PRJCONF_UART_RX_PIN NRF_GPIO_PIN_MAP(0, 3)
/** \brief 串口发送引脚 */
#define PRJCONF_UART_TX_PIN NRF_GPIO_PIN_MAP(0, 28)
/** \brief 串口数据方向控制引脚，高电平发送使能，低电平接收使能 */
#define PRJCONF_UART_DE     NRF_GPIO_PIN_MAP(0, 2)

/** \brief 串口接收使能 */
#define PRJCONF_UART_RX_ENABLE nrf_gpio_pin_clear(PRJCONF_UART_DE)

/** \brief 串口发送使能 */
#define PRJCONF_UART_TX_ENABLE nrf_gpio_pin_set(PRJCONF_UART_DE)

/** \brief modbus帧间T35超时时间，波特率大于19200时，T35=1750us，波特率小于等于19200时，T35=3.5*10/baudrate */
#define PRJCONF_MODBUS_T35 (2) //< 单位:ms


/******************************************************************************
 **************************** DRV8432 config **********************************
 *****************************************************************************/

/** \brief 编码器输出A，doc:WFMControl.pdf  */
#define PRJCONF_DRV8432_ENC_A_PIN   NRF_GPIO_PIN_MAP(0, 15)

/** \brief 编码器输出B */
#define PRJCONF_DRV8432_ENC_B_PIN   NRF_GPIO_PIN_MAP(0, 14)

/** \brief PWM_A输入引脚*/
#define PRJCONF_DRV8432_PWMA_PIN    NRF_GPIO_PIN_MAP(0, 19)

/** \brief PWM_B输入引脚 */
#define PRJCONF_DRV8432_PWMB_PIN    NRF_GPIO_PIN_MAP(0, 21)

/** \brief RESET_AB引脚 */
#define PRJCONF_DRV8432_RESETAB_PIN NRF_GPIO_PIN_MAP(0, 20)

/** \brief RESET_CD引脚 */
#define PRJCONF_DRV8432_RESETCD_PIN NRF_GPIO_PIN_MAP(0, 22)

/** \brief OTW(过热,低有效)引脚 */
#define PRJCONF_DRV8432_OTW_PIN     NRF_GPIO_PIN_MAP(0, 17)

/** \brief FAULT(错误信号,低有效)引脚 */
#define PRJCONF_DRV8432_FAULT_PIN   NRF_GPIO_PIN_MAP(0, 16)

/** \brief PWM输出频率 */
#define PRJCONF_DRV8432_PWM_FREQ                (40000L)

/** \brief 复位保持时间，单位:ms */
#define PRJCONF_DRV8432_RESET_HOLDING_TIME      (500)

/** \brief 编码器采样定时器周期,单位ms */
#define PRJCONF_DRV8432_ENCODER_SAMPLE_TIME     (50)

#if PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_GNL_32_21E

/** \brief 减速比 */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (32)

/** \brief 编码器分辨率CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_0

/** \brief 减速比 */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (24)

/** \brief 编码器分辨率CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (480)


#elif PRJCONF_CURRENT_WFM_TYPE == WFM_TYPE_FWF_840_1

/** \brief 减速比 */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (50)

/** \brief 编码器分辨率CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#else /* default config */

/** \brief 减速比 */
#define PRJCONF_DRV8432_ENCODER_REDUCTION_RATIO (32)

/** \brief 编码器分辨率CPR */
#define PRJCONF_DRV8432_ENCODER_RESOLUTION      (120)

#endif

/** \brief 直径 */
#define PRJCONF_DRV8432_MOTOR_DIAMETER          (40)

/******************************************************************************
 **************************** modbus slave config ************************
 *****************************************************************************/
#define PRJCONF_MODBUS_SLAVE_BLE_CACHE_SIZE (512)  /**< \brief MODBUS从机ble接收数据缓存大小 */
#define PRJCONF_MODBUS_SLAVE_UART_CACHE_SIZE (512) /**< \brief MODBUS从机uart接收数据缓存大小 */

/******************************************************************************
 **************************** event config ************************************
 *****************************************************************************/


#endif /* __PRJCONFIG_H */
