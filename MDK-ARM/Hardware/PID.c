#include "PID.h"

	
#define Middle_Angle 1.5
int MOVE = 30;  //加减速的基础伏值

extern float Roll,Pitch,Yaw; 
extern short encoder1;
extern short encoder2;

float Balance_Kp=-270,Balance_Kd=-1.26;   
float Velocity_Kp=-190,Velocity_Ki=-0.95;  
float Turn_Kp=-40,Turn_Kd=0.6;           
uint8_t Flag_Stop = 0;    
extern uint8_t Flag_Balance;

extern uint8_t Forward_Flag, Backward_Flag, TurnLeft_Flag, TurnRight_Flag;
extern uint8_t Flag_Avoid;          
extern float distance; 
//直立环
int Balance(float Angle,float Gyro)
{
	float Angle_bias,Gyro_bias;
	int balance;
	Angle_bias= Middle_Angle - Angle;
	Gyro_bias=-Gyro;
	balance = Balance_Kp * Angle_bias + Balance_Kd * Gyro_bias;
	return balance;
}


//速度环

int Velocity (int encoder_left,int encoder_right)
{
	static float Encoder_Least,Encoder_bias;
	static float velocity;
	static float Movement;
	static float Encoder_Integral;
	
	if(Forward_Flag == 0 && Backward_Flag == 0) Movement = 0;
	if(Forward_Flag == 1) Movement = MOVE;
	else if(Backward_Flag == 1) Movement = -MOVE;
	
	if(Flag_Avoid && distance < 30) Movement = -MOVE;
	
	Encoder_Least=0-(encoder_left+encoder_right);
	Encoder_bias*=0.7;
	Encoder_bias+=Encoder_Least*0.3;
	
	Encoder_Integral+=Encoder_bias;
	Encoder_Integral=Encoder_Integral+Movement;
	if(Encoder_Integral>10000)
	{
		Encoder_Integral=10000;
	}
	if(Encoder_Integral<-10000)
	{
		Encoder_Integral=-10000;
	}
	
	
	if(Flag_Stop==1)
	{
		Encoder_Integral=0;
	}
	
	velocity=Velocity_Kp*Encoder_bias+Velocity_Ki*Encoder_Integral;
	return velocity;
}

//转向环

int Turn(float gyro)
{
	static float Turn_Target,turn;
	float Kp=Turn_Kp,Kd;
	
	if(TurnLeft_Flag == 1)   Turn_Target = MOVE / 2;
	else if(TurnRight_Flag == 1)  Turn_Target = -MOVE / 2;
	else Turn_Target = 0;
	
	if(TurnLeft_Flag == 0 && TurnRight_Flag == 0)
	{
			Kd = Turn_Kd;  
	}
	else if(TurnLeft_Flag == 1 || TurnRight_Flag == 1)
	{
			Kd = 0;
	}

	turn=Kp*Turn_Target+Kd*gyro;//偏航角
	
	return turn;
}

void Control()
{
	Read_DMP();
	encoder1=__HAL_TIM_GET_COUNTER(&htim3);
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	encoder2=__HAL_TIM_GET_COUNTER(&htim4);
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	
	int BalancePWM=Balance(Pitch,gyro[0]);
	int VelocityPWM=Velocity(encoder1,encoder2);
	int TurnPWM=Turn(gyro[2]);
	if(TurnLeft_Flag == 0 && TurnRight_Flag == 0)
	{
			Turn_Kd = 0.6;  
	}
	else if(TurnLeft_Flag == 1 || TurnRight_Flag == 1)
	{
			Turn_Kd = 0;
	}
	
	int LeftPWM=BalancePWM+VelocityPWM+TurnPWM;
	int RightPWM=BalancePWM+VelocityPWM-TurnPWM;
	
	if(abs(encoder1) > 40 || abs(encoder2) > 40) Flag_Stop = 1; 
	
	if(Pitch>70||Pitch<-70) {Flag_Stop=1;}
	else {Flag_Stop=0;}
	
	if(Flag_Balance==1&&Flag_Stop == 0)
	{
		Set_PWM(LeftPWM,RightPWM);
	}
	else
	{
		Set_PWM(0,0);
	}
	
}

