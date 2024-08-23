#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "Uart_Init.h"
#include "Motormove.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "TML_Register.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


//#define REPORT_LENGTH_1  1
//#define REPORT_LENGTH_12 12
//#define REPORT_LENGTH_15 15




bool ack_received_flag = false;
bool ack_waiting_flag = false;
bool handle_flag =  false;
uint8_t rdata_info[12];


uint8_t rdata_byte;
uint8_t rdata_buf[UART_RX_BUF_SIZE] = {0};
uint8_t re_index = 0;	


APP_TIMER_DEF(ID_Receive_time);
#define TIME_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2)


void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code;
		app_timer_init();
	err_code = app_timer_create(&ID_Receive_time,APP_TIMER_MODE_SINGLE_SHOT,timeout_handler);
	APP_ERROR_CHECK(err_code);
	
//	app_timer_start(ID_Receive_time,TIME_LEVEL_MEAS_INTERVAL,NULL);

}

enum state {
    UART_STATE_IDLE,      /**< \brief 空闲状态 */
    UART_STATE_SENDING,   /**< \brief 发送状态 */
    UART_STATE_RECEIVING  /**< \brief 接收状态 */
};
enum state uart_state = UART_STATE_IDLE;


void rdata_handling(uint8_t* data,uint16_t length){
	if(length == 15 && data[0] == 0x4F && data[1]== 0x0C){
		NRF_LOG_INFO("Receive 15-bytes message.		addr: %02X%02X, value: %02X%02X%02X%02X",data[8],data[9],data[12],data[13],data[10],data[11])
		Motor_drv.addr = (data[8] << 8) | data[9];
		Motor_drv.value = (data[12]<<24) | (data[13]<<16) | (data[10]<<8) | data[11];
		TML_Register_match(&Motor_drv.addr,&Motor_drv.value);
//		TML_Value_of_Register_CALC(&Motor_drv.value);
	}

	else if(length == 1 && data[0] == 0x4F){
		NRF_LOG_INFO("Receive single byte: 4F");
	}
	else if(length == 13 && data[0] == 0x4F && data[1] == 0x0A){
		NRF_LOG_INFO("Receive 13-bytes message.  	addr: %02X%02X, value: %02X%02X",data[8],data[9],data[10],data[11]);
	}else{
		NRF_LOG_INFO("Wrong Info");
	}
	NRF_LOG_FLUSH();
}

//超时处理函数   
 void timeout_handler(void *p_context){
	
	 UNUSED_PARAMETER(p_context);
	 rdata_handling(rdata_buf, re_index);
	 re_index = 0;
	 uart_state = UART_STATE_IDLE;
	
}




void uart_Interrupt_handle(app_uart_evt_t * p_event)
{
	uint32_t err_code;
	uint8_t i;
	switch(p_event->evt_type){

		case APP_UART_FIFO_ERROR:
				APP_ERROR_HANDLER(p_event->data.error_code);
		break;
		
			
		case APP_UART_DATA_READY:		//串口接收事件
			if (uart_state == UART_STATE_RECEIVING) {
				err_code = app_uart_get(&rdata_byte);
				APP_ERROR_CHECK(err_code);
				rdata_buf[re_index++] = rdata_byte;
				app_timer_stop(ID_Receive_time);
				app_timer_start(ID_Receive_time,TIME_LEVEL_MEAS_INTERVAL,NULL);
			} else {
			    err_code = app_uart_get(&rdata_byte);
			}
			break;
		
//			字节间开启计时器，每次接收一个开启一个，再接收则关闭，再开启，如果超过1ms就判定此段信息结束，下一段准备接收
		
//		if(uart_state == UART_STATE_IDLE ){
//			uart_state = UART_STATE_RECEIVING;
//			app_timer_start(ID_Receive_time,TIME_LEVEL_MEAS_INTERVAL,NULL);
//		}else{
//			app_timer_stop(ID_Receive_time);
//			app_timer_start(ID_Receive_time,TIME_LEVEL_MEAS_INTERVAL,NULL);		
//		}
		
		
////		(uint8_t)NRF_UART0->RXD
//		if(re_index == 15){
////			for(i = 0;i<15;i++){
////				UART_WriteData(&rdata_buf[i],1);	
////			}
//		NRF_LOG_INFO("addr:%02X %02X, value: %02X %02X %02X %02X",rdata_buf[8],rdata_buf[9],rdata_buf[12],rdata_buf[13],rdata_buf[10],rdata_buf[11]);

////		UART_WriteData(rdata_buf, re_index);
//		re_index = 0;	
//		}
//		break;
		
		
		
    case APP_UART_TX_EMPTY:			//串口发送完成事件
			PRJ_CONF_UART_RX_ENABLE;
		
		  uart_state = UART_STATE_RECEIVING;
		break;		
		
		default:
			break;
	}
}

#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED




void init_gpio(void)
{
  nrf_gpio_cfg_output(PRJ_CONF_UART_DE);
}
void UART_Init_9600()
{
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          PRJ_CONF_UART_RX_PIN,
          PRJ_CONF_UART_TX_PIN,
          UART_PIN_DISCONNECTED,
          UART_PIN_DISCONNECTED,
          UART_HWFC,
          false,
					UART_BAUDRATE_BAUDRATE_Baud9600
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_Interrupt_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

}
void UART_Init_115200()
{
	uint32_t err_code;
    const app_uart_comm_params_t comm_params =
      {
          PRJ_CONF_UART_RX_PIN,
          PRJ_CONF_UART_TX_PIN,
          UART_PIN_DISCONNECTED,
          UART_PIN_DISCONNECTED,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
          NRF_UART_BAUDRATE_115200
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_Interrupt_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

}

void UART_WriteData(uint8_t *pData, uint8_t dataLen)
{
	uint16_t i;
	PRJ_CONF_UART_TX_ENABLE;
	for(i = 0; i < dataLen; i++)
	{
		app_uart_put(pData[i]);
	}
	nrf_delay_ms(30);
}



