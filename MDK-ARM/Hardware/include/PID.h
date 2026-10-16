#ifndef __PID_H
#define __PID_H


#include "mpu6050.h"
#include "encoder.h"
#include "tim.h"
#include "motor.h"


void Control(void);
int Turn(float gyro);
int Velocity (int encoder_left,int encoder_right);
int Balance(float Angle,float Gyro);
	
#endif



