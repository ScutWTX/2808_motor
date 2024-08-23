#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_uart.h"
#include "app_error.h"
#include "app_timer.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "Uart_Init.h"
#include "Motormove.h"
#include "TML_Register.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define M_PI       3.14159265358979323846
#define RPM_sf     9.5492965855137
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

motor_drv Motor_drv;

	float No_encoder_lines = 29.23208678f; 		//编码器行数
	float Tr = 35.03f;													//传动比
	float Ts = 0.001f;													//采样时间

void TML_Register_match(uint16_t* addr,uint32_t* value){
	
	float POS_SI;
	float POS_rot;
	uint32_t pos_rot_int; 
	uint32_t pos_si_int;

	float SPD_SI;
	float spd_si_integer;	
	float spd_si_fractional;	



	switch(*addr){
		case 0x02A2: 

		NRF_LOG_INFO("Command Acceleration of motor:  rad/s^2 ");
		break;
		
		case 0x02A0: 
//			NRF_LOG_INFO("Command Speed of motor: ");
		
		spd_si_integer = (*value) >> 16;
		spd_si_fractional = (float)((*value) &0xFFFF)/65536.0f;
		SPD_SI = ((spd_si_integer + spd_si_fractional) * 2.0f* M_PI)/(4.0f*No_encoder_lines*Tr*Ts) ;
		
		NRF_LOG_INFO("Current Speed of motor: %d.%04d rad/s  %d rpm", (int)SPD_SI, (int)(SPD_SI*10000)%10000,(uint16_t)round(SPD_SI*RPM_sf));

		break;
		
		case 0x0228: 
		POS_SI = (*value)*360 / (4*No_encoder_lines*Tr*Tr);  //标准单位度数degree (由UI换算成rot  再除以传动比  乘以360°)
		pos_si_int = POS_SI * 10000;
		POS_rot = POS_SI / 360.0f;  
		pos_rot_int = (uint32_t)(POS_rot * 100);  // 乘以100以保留两位小数
		NRF_LOG_INFO("Current Position:%d.%04d		%d.%02d rot", pos_si_int/10000,pos_si_int%10000,pos_rot_int/100,pos_rot_int%100);
		break;
		
//		case 0x022C:
//		spd_si_integer = (*value) >> 16;
//		spd_si_fractional = (float)((*value) &0xFFFF)/65536.0f;
//		SPD_SI = ((spd_si_integer + spd_si_fractional) * 2.0f* M_PI)/(4.0f*No_encoder_lines*Tr*Ts);
//		NRF_LOG_INFO("Current Speed of motor: %d.%04d rad/s  %d rpm", (int)SPD_SI, (int)(SPD_SI*10000)%10000,(uint16_t)round(SPD_SI*RPM_sf));

//		break;
		
		
		case 0x029E: 
		
		POS_SI = (*value)*360 / (4*No_encoder_lines*Tr*Tr);  //标准单位度数degree (由UI换算成rot  再除以传动比  乘以360°)
		
		pos_si_int = POS_SI * 10000;
		NRF_LOG_INFO("Command Position:%d.%04d", pos_si_int/10000, pos_si_int%10000);
		break;
		
		case 0x0273: NRF_LOG_INFO("KII of motor ");
		break;
		case 0x0271: NRF_LOG_INFO("KPI of motor ");
		break;
		
		case 0x090F: NRF_LOG_INFO("SRH of motor ");
		break;
		case 0x090E: NRF_LOG_INFO("SRL of motor ");
		break;
	
		
	}

	NRF_LOG_FLUSH();
	
	
}

//void TML_Value_of_Register_CALC(uint16_t* value){
//	float No_encoder_lines = 29.23208678; 		//编码器行数
//	float Tr = 35.03;													//传动比
//	float Ts = 0.001;													//采样时间
//	float POS_SI;
//	POS_SI = (*value)*360 / (4*No_encoder_lines*Tr*Tr);  //标准单位度数degree (由UI换算成rot  再除以传动比  乘以360°)
//	NRF_LOG_INFO("Current Position:%f",POS_SI);
//	

//	
//	
//	
//	
//}

