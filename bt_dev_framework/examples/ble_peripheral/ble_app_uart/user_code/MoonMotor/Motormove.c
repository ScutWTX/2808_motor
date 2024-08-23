#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "Uart_Init.h"
#include "Motormove.h"
#include "TML_Register.h"
#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#define M_PI       3.14159265358979323846


uint16_t Rotation_Speed_rpm;
uint16_t ACC_rad_s2;
uint16_t Position_Motor_rot;


bool Motor_Stop_flag = 0;
bool Motor_Axisoff_flag = 0;
bool Motor_UPD_flag = 0;

bool CPR_flag = 0;
uint16_t result;
uint8_t M_state_flag = 1;


enum Motor_move_state {
    MOTOR_STATE_SPEED,      /**< \brief 速度模式 */
    MOTOR_STATE_POSITION,   /**< \brief 位置模式 */
    MOTOR_STATE_TORQUE 		 	/**< \brief 转矩模式 */
};
enum Motor_move_state Motor_mstate = MOTOR_STATE_SPEED;

void __Motor_state(uint16_t rpm,uint16_t acc,uint16_t rot){
	if(M_state_flag == 1){
		SPEED_MODE(rpm,acc);
	}else if(M_state_flag == 2){
		POSITION_MODE(rpm,acc,rot);
	}else if(M_state_flag == 3){
		TORQUE_MODE();
	}	
}

void SPEED_MODE(uint16_t rpm,uint16_t acc){
	Motor_mstate = MOTOR_STATE_SPEED;
	Motor_CSPD(rpm);
	Motor_CACC(acc);
	Motor_MODE_SP();
	Motor_TUM1();
}
void POSITION_MODE(uint16_t rpm,uint16_t acc,uint16_t rot){
	Motor_mstate = MOTOR_STATE_POSITION;
	Motor_CSPD(rpm);
	Motor_CACC(acc);
//	if(CPR_flag == 0)
//	{
//		Motor_CPOS(30);
//		Motor_CPR();
//		CPR_flag = 1;
//	}
	Motor_CPOS(rot);
	Motor_CPA();
	Motor_Mode_PP();
	Motor_TUM1();
}

void TORQUE_MODE(){
	Motor_mstate = MOTOR_STATE_TORQUE;		


}

void Motor_Baudrate(void){
		uint8_t msg[] = {0x06, 0x00, 0xF0, 0x08, 0x20, 0x00, 0x04, 0x22};	//电机波特率115200
		UART_WriteData(msg, sizeof msg);
		
}

void Motor_Axison(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x01, 0x02, 0xF7};	//电机使能
		UART_WriteData(msg, sizeof msg);

}
		

void Motor_Axisoff(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x00, 0x02, 0xF6};	//电机失能
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
//		while(ack_received_flag != true){
//		nrf_delay_ms(ack_wait_time_ms);
//			}
}

void Motor_Reset(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x04, 0x02, 0xFA};	//电机复位
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
//		while(ack_received_flag != true){
//		nrf_delay_ms(ack_wait_time_ms);
//			}	
}

void Motor_UPD(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x01, 0x08, 0xFD};	//立即执行 
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
//		while(ack_received_flag != true){
//		nrf_delay_ms(ack_wait_time_ms);
//			}	
	
}
/*等待 上一段运动完成后执行*/
void Motor_UPD__(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x00, 0x08, 0xFC};	
		UART_WriteData(msg, sizeof msg);

	
}

void Motor_Stop(void){
		uint8_t msg[] = {0x04, 0x00, 0xF0, 0x01, 0xC4, 0xB9};	//停止 
//		ack_received_flag = false;
			UART_WriteData(msg, sizeof msg);		
//	while(ack_received_flag != true){
//		nrf_delay_ms(ack_wait_time_ms);
//			}	
}

void Motor_TUM1(void){
		uint8_t msg[] = {0x08, 0x00, 0xF0, 0x59, 0x09, 0xFF, 0xFF, 0x40, 0x00, 0x98};	//TUM模式 
//		ack_received_flag = false;
		UART_WriteData(msg, sizeof msg);
//		while(ack_received_flag != true){
//		nrf_delay_ms(ack_wait_time_ms);
//			}	
}

void Motor_CACC(uint16_t rad_s2){
		uint8_t msg[10] = {0x08, 0x00, 0xF0, 0x24, 0xA2};	//设定加速度
		float ACC_IU;
		ACC_IU = rad_s2*4*No_encoder_lines*Tr*Ts*Ts/(2*M_PI);
		uint16_t integer_part = (uint16_t) ACC_IU;
		msg[7] = (integer_part >> 8) & 0xFF;
		msg[8] = integer_part & 0xFF;
		float fractional_part = ACC_IU - integer_part;
		result = (uint16_t)round(fractional_part * 65536);
		msg[5] = (result >> 8) & 0xFF;
		msg[6] = (result) & 0xFF;
		uint8_t Checksum = 0;
		for(uint8_t i = 0;i<10;i++){
				Checksum += msg[i];
		}
		msg[9] = Checksum & 0xFF;
		UART_WriteData(msg, sizeof msg);

}

void Motor_CSPD(uint16_t rpm){
		uint8_t msg[10] = {0x08, 0x00, 0xF0, 0x24, 0xA0} ;	//目标速度  设置操作位、轴ID、寄存器地址 
		float Ro_Spd_IU;
		Ro_Spd_IU = (rpm) * 4*No_encoder_lines*Tr*Ts/(9.5492965855137*2*M_PI);
		
    // 保留整数部分并转换为uint16_t
    uint16_t integer_part = (uint16_t)Ro_Spd_IU;
		
    // 将整数部分分解为两个字节存储到msg[8]和msg[9]
    msg[7] = (integer_part >> 8) & 0xFF; // 高8位
    msg[8] = integer_part & 0xFF;        // 低8位
		float fractional_part = Ro_Spd_IU - integer_part;
		result = (uint16_t)round(fractional_part * 65536);
		msg[5] = (result >> 8) & 0xFF;
		msg[6] = (result) & 0xFF;
		uint8_t Checksum = 0;
		for(uint8_t i = 0;i<10;i++){
				Checksum += msg[i];
		}
		msg[9] = Checksum & 0xFF;
		UART_WriteData(msg, sizeof msg);
		
}
//void Motor_CPOS(void){
//		
//uint8_t msg[] = {0x08, 0x00, 0xF0, 0x24, 0x9E, 0xA0, 0x00, 0x00, 0x06, 0x60};  		//目标位置 

//		UART_WriteData(msg, sizeof msg);
//}
void Motor_CPOS(float rot){
	uint8_t msg[10] = {0x08, 0x00, 0xF0, 0x24, 0x9E};  		//目标位置 
	uint16_t POS_IU;
	POS_IU = round(rot*Tr*No_encoder_lines*4.0f);
	msg[7] = (POS_IU >> 24) &0xFF;
	msg[8] = (POS_IU >> 16) &0xFF;
	msg[5] = (POS_IU >> 8) &0xFF;
	msg[6] = (POS_IU) &0xFF;
	uint8_t Checksum = 0;
	for(uint8_t i = 0;i<10;i++){
				Checksum += msg[i];
		}
		msg[9] = Checksum & 0xFF;
		UART_WriteData(msg, sizeof msg);
		
}

/* 相对位置，位置模式叠加*/
void Motor_CPR(void){
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0x59, 0x09, 0xDF, 0xFF, 0x00, 0x00, 0x38};
    UART_WriteData(msg, sizeof(msg));
}
/* 绝对位置，位置模式绝对*/
void Motor_CPA(void){
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0x59, 0x09, 0xFF, 0xFF, 0x20, 0x00, 0x78};
    UART_WriteData(msg, sizeof(msg));
}

void Motor_Mode_PP(void){
    uint8_t msg[] = {0x08, 0x00, 0xF0, 0x59, 0x09, 0xBF, 0xC1, 0x87, 0x01, 0x62};
    UART_WriteData(msg, sizeof(msg));
}


void Motor_MODE_SP(void){
	uint8_t msg[] = {0x08, 0x00, 0xF0, 0x59, 0x09, 0xBB, 0xC1, 0x83, 0x01, 0x5A};
UART_WriteData(msg, sizeof msg);

}

void Motor_MC(void){
    uint8_t msg[] = {0x04, 0x00, 0xF0, 0x70, 0x0F, 0x73};
    UART_WriteData(msg, sizeof(msg));
}

void Motor_WAIT(void){
    uint8_t msg[] = {0x04, 0x00, 0xF0, 0x04, 0x08, 0x00};
    UART_WriteData(msg, sizeof(msg));
}




//等待命令发送stop
void Motor_Stop_command(void){
	if(Motor_Stop_flag)
	{
		Motor_Stop();
		nrf_delay_ms(1000);
	}
//	Motor_Stop_flag = false;
}

void Motor_Axisoff_command(void){
	if(Motor_Axisoff_flag)
	{
		Motor_Axisoff();
	}
	Motor_Axisoff_flag = false;
}
//等待发送启动命令
void Motor_UPD_command(void){
	if(Motor_UPD_flag)
	{
		Motor_UPD();
	}
	Motor_UPD_flag = false;
}