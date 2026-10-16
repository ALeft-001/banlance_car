#include "encoder.h"



int Read_Encoder(uint8_t tim)
{
	int encoder;
	if(tim==3)
	{
		encoder=(short)__HAL_TIM_GetCounter(&htim3);
		__HAL_TIM_SetCounter(&htim3,0);
		
	}
	else if(tim==4)
	{	
		encoder=(short)__HAL_TIM_GetCounter(&htim4);
		__HAL_TIM_SetCounter(&htim4,0);
	}
	else
	{
		encoder=0;
	}
	return encoder;
}






