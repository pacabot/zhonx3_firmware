/**************************************************************************/
/*!
    @file     motors.h
    @author   PLF Pacabot.com
    @date     01 December 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __MOTORS_H__
#define __MOTORS_H__

typedef struct
{
	uint32_t IN1;
	uint32_t IN2;
} motor;

#define MOTORS_TIMER		htim8

#define DIRECTION_FORWARD	1
#define DIRECTION_BACKWARD	0
#define DECAY_SLOW			1
#define DECAY_FAST			0

#define MOTORS_STANDBY 	GPIO_PIN_11

#define LEFT_MOTOR_GPIO_IN1 	GPIO_PIN_6
#define LEFT_MOTOR_GPIO_IN2 	GPIO_PIN_7
#define RIGHT_MOTOR_GPIO_IN1 	GPIO_PIN_8
#define RIGHT_MOTOR_GPIO_IN2 	GPIO_PIN_9

#define LEFT_MOTOR_IN1 		TIM_CHANNEL_1
#define LEFT_MOTOR_IN2 		TIM_CHANNEL_2
#define RIGHT_MOTOR_IN1 	TIM_CHANNEL_3
#define RIGHT_MOTOR_IN2 	TIM_CHANNEL_4

void Motors_Pwm_Init(void);
void Motors_Enable_Driver(char isOn);
void Motor_Set(motor *mot, int isForward, int duty, int isSlowDecay);
void Motors_Brake(void);
void Motors_Test(void);

#endif
