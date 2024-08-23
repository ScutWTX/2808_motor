#ifndef __MOTORMOVE
#define __MOTORMOVE
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define ack_wait_time_ms 1


extern uint16_t Rotation_Speed_rpm;
extern uint16_t ACC_rad_s2;
extern uint16_t Position_Motor_rot;

extern bool Motor_Stop_flag;
extern bool Motor_Axisoff_flag;
extern bool Motor_UPD_flag;
extern uint16_t result;
extern uint8_t M_state_flag;

void __Motor_state(uint16_t,uint16_t,uint16_t);
void SPEED_MODE(uint16_t,uint16_t);
void POSITION_MODE(uint16_t,uint16_t,uint16_t);
void TORQUE_MODE(void);

void Motor_Baudrate(void);
void Motor_Axison(void);
void Motor_Axisoff(void);
void Motor_Reset(void);
void Motor_UPD(void);
void Motor_UPD__(void);
void Motor_Stop(void);

void Motor_TUM1(void);
void Motor_CACC(uint16_t);
void Motor_CSPD(uint16_t);
void Motor_MODE_SP(void);
//void Motor_Mode(void);
void Motor_Stop_command(void);
void Motor_Axisoff_command(void);
void Motor_UPD_command(void);

void Motor_CPOS(float);
void Motor_CPR(void);
void Motor_CPA(void);


void Motor_Mode_PP(void);
void Motor_MC(void);
void Motor_WAIT(void);



#endif
