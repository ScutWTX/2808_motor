#ifndef __TML_Register
#define __TML_Register
#include <stdint.h>
#include <stdbool.h>

void TML_Register_match(uint16_t*,uint32_t*);
//void TML_Value_of_Register_CALC(uint16_t*);

typedef struct {
	uint16_t addr;
	uint32_t value;	
}motor_drv;

extern float No_encoder_lines; 							//编码器行数
extern float Tr;														//传动比
extern float Ts;														//采样时间

extern motor_drv Motor_drv;
#endif
