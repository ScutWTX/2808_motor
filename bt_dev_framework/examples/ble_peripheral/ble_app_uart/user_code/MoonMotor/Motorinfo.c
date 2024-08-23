#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "Uart_Init.h"
#include "Motormove.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

void Motor_CSPD_inquire()
{

		uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x05, 0x00, 0xF1, 0x02, 0xA0, 0x40};
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
	
}

void Motor_CACC_inquire()
{

		uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x05, 0x00, 0xF1, 0x02, 0xA2, 0x42};
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
	
}

void Motor_SRH_inquire()
{
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x04, 0x00, 0xF1, 0x09, 0x0F, 0xB5};
    UART_WriteData(msg, sizeof(msg));
}

void Motor_SRL_inquire()
{
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x04, 0x00, 0xF1, 0x09, 0x0E, 0xB4};
    UART_WriteData(msg, sizeof(msg));
}

void Motor_APOS_inquire(){
	
	uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x05, 0x00, 0xF1, 0x02, 0x28, 0xC8};
    UART_WriteData(msg, sizeof(msg));
}

void Motor_ASPD_inquire(){
	
	uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x05, 0x00, 0xF1, 0x02, 0x2C, 0xCC};
    UART_WriteData(msg, sizeof(msg));
	
}
void Motor_CPOS_inquire()
{
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0xB0, 0x05, 0x00, 0xF1, 0x02, 0x9E, 0x3E};
    UART_WriteData(msg, sizeof(msg));
}


//  电机状态查询，循环访问

void Motor_info_inquire(){
	if(Motor_Stop_flag == 0){
//			Motor_SRH_inquire();
//			nrf_delay_ms(300);
			Motor_ASPD_inquire();
			nrf_delay_ms(300);
//		  Motor_CACC_inquire();
//			nrf_delay_ms(300);
//			Motor_CPOS_inquire();
//			nrf_delay_ms(300);
			Motor_APOS_inquire();
			nrf_delay_ms(300);
//			
	
	}
}

