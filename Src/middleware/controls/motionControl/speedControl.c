/**************************************************************************/
/*!
    @file    speedControl.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/motors/motors.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/speedControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */
speed_control_struct speed_control;
CONTROL_DEF speed_control_pid;
arm_pid_instance_f32 encoder_pid_instance;

int speedControl_Init(void)
{
	int rv;
	encoder_pid_instance.Kp = 0.001;
	encoder_pid_instance.Ki = 0;//0.000001;//0.1;
	encoder_pid_instance.Kd = 0;//0.4;

	speed_control.old_cnt = 0;
	speed_control.current_cnt = 0;
	speed_control.maintain_cnt = 0;
	speed_control.old_speed2 = 0;
	speed_control.current_speed2 = 0;
	speed_control.maintain_speed = 0;
	speed_control.step_distance = 0;
	speed_control.mm_distance = 0;
	speed_control.speed_consigne = 0;
	speed_control.speed.pid_instance = &encoder_pid_instance;

	pidControllerInit(speed_control.speed.pid_instance);


	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControl(void)
{
	int rv;
	float current_cnt;
	float speed_error;
	int get_correction;
	//	int consigne = 20;

	static unsigned int Pulses[2] = {0,0};

	//current_cnt = speed_control.old_cnt - (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder) / 2);
	current_cnt = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder) / 2);

	speed_control.old_cnt = current_cnt;
	speed_control.current_speed = (current_cnt * HI_TIME_FREQ);
	//	speed_control.speed_consigne += (ACCELERATION / (float)HI_TIME_FREQ);

	//	speed_error = speed_control.speed_consigne - (float)speed_control.current_speed;
	speed_error = current_cnt;

	get_correction = pidController(speed_control.speed.pid_instance, speed_error);

//	Pulses[1] += get_correction;
//	Pulses[0] += get_correction;
	Pulses[1] += get_correction;
	Pulses[0] += get_correction;

	if (current_cnt < 0)
	{
		Pulses[1] = (current_cnt * -1);
		Pulses[0] = (current_cnt * -1);
		motorSet(&left_motor, DIRECTION_FORWARD, Pulses[0], DECAY_FAST);
		motorSet(&right_motor, DIRECTION_FORWARD, Pulses[1], DECAY_FAST);
	}
	else
	{
		Pulses[1] = current_cnt;
		Pulses[0] = current_cnt;
		motorSet(&left_motor, DIRECTION_BACKWARD, Pulses[0], DECAY_FAST);
		motorSet(&right_motor, DIRECTION_BACKWARD, Pulses[1], DECAY_FAST);
	}

	return SPEED_CONTROL_E_SUCCESS;
}

int speedAcc(uint32_t initial_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedDcc(uint32_t final_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedMaintain(uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}