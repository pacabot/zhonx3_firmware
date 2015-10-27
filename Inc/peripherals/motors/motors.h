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

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MOTORS_DRIVER_E_SUCCESS  0
#define MOTORS_DRIVER_E_ERROR    MAKE_ERROR(MOTORS_DRIVER_MODULE_ID, 1)

typedef struct
{
	uint32_t IN1;
	uint32_t IN2;
} motor;

#define MOTORS_TIMER		htim8

#define DECAY_SLOW			0
#define DECAY_FAST			1

#define MOTORS_STANDBY 	GPIO_PIN_11

#define LEFT_MOTOR_GPIO_IN1 	GPIO_PIN_6
#define LEFT_MOTOR_GPIO_IN2 	GPIO_PIN_7
#define RIGHT_MOTOR_GPIO_IN1 	GPIO_PIN_8
#define RIGHT_MOTOR_GPIO_IN2 	GPIO_PIN_9

#define LEFT_MOTOR_IN1 		TIM_CHANNEL_1
#define LEFT_MOTOR_IN2 		TIM_CHANNEL_2
#define RIGHT_MOTOR_IN1 	TIM_CHANNEL_3
#define RIGHT_MOTOR_IN2 	TIM_CHANNEL_4

extern motor left_motor;
extern motor right_motor;

void motorsInit(void);
void motorsDriverSleep(int isOn);
void motorSet(motor *mot, int duty, int isSlowDecay);
void motorsBrake(void);
void motorsTest(void);

#endif
