#include "motor.h"



void Set_PWM(int left, int right)
{
	if(left>7200)
	{
		left=7200;
	}
	if(left<-7200)
	{
		left=-7200;
	}
	if(right>7200)
	{
		right=7200;
	}
	if(right<-7200)
	{
		right=-7200;
	}
	
	
	
	
	if(right>0)//正 
	{
		HAL_GPIO_WritePin(AIN_1_GPIO_Port,AIN_1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(AIN_2_GPIO_Port,AIN_2_Pin,GPIO_PIN_RESET);
	}
	else//反
	{
		HAL_GPIO_WritePin(AIN_1_GPIO_Port,AIN_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AIN_2_GPIO_Port,AIN_2_Pin,GPIO_PIN_SET);
	}
	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,abs(right));
	
	
	if(left>0)//正 
	{
		HAL_GPIO_WritePin(BIN_1_GPIO_Port,BIN_1_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(BIN_2_GPIO_Port,BIN_2_Pin,GPIO_PIN_SET);
	}
	else//反
	{
		HAL_GPIO_WritePin(BIN_1_GPIO_Port,BIN_1_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(BIN_2_GPIO_Port,BIN_2_Pin,GPIO_PIN_RESET);
	}
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,abs(left));
}


