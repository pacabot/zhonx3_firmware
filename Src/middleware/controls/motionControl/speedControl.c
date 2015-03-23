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
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

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
	encoder_pid_instance.Kp = 700.0;
	encoder_pid_instance.Ki = 0;//0.000001;//0.1;
	encoder_pid_instance.Kd = 800;//0.4;

	speed_control.current_distance = 0;
	speed_control.gap_distance_per_loop = 0;
	speed_control.old_distance = 0;
	speed_control.current_speed = 0;

	speed_control.speed_error = 0;
	speed_control.speed_command = 0;
	speed_control.speed_consigne = 0;

	speed_control.distance_consigne = 0;

	speed_control.speed.pid_instance = &encoder_pid_instance;

	pidControllerInit(speed_control.speed.pid_instance);


	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControl(void)
{
	//	int consigne = 20;

	speed_control.current_distance = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder)) / 2;

	speed_control.gap_distance_per_loop = speed_control.current_distance - speed_control.old_distance;	//delta distance per loop
	speed_control.current_speed = (speed_control.gap_distance_per_loop * (float)HI_TIME_FREQ);			//actual speed (mm/s)
	//	speed_control.speed_error = speed_control.speed_consigne - speed_control.current_speed;				//for speed control

	speed_control.speed_consigne += (ACCELERATION / (float)HI_TIME_FREQ);								//speed consigne (mm/s)
	speed_control.distance_consigne += (speed_control.speed_consigne / (float)HI_TIME_FREQ);

	speed_control.speed_error = speed_control.distance_consigne - speed_control.current_distance;		//for distance control

	speed_control.speed_command = pidController(speed_control.speed.pid_instance, speed_control.speed_error);

	speed_control.old_distance = speed_control.current_distance;

	return SPEED_CONTROL_E_SUCCESS;
}

int speedAcc(uint32_t initial_speed, uint32_t distance)
{
	return SPEED_CONTROL_E_SUCCESS;
}

int speedDcc(uint32_t final_speed, uint32_t distance)
{
	return SPEED_CONTROL_E_SUCCESS;
}

int speedMaintain(float speed)
{
	return SPEED_CONTROL_E_SUCCESS;
}

void speedControlTest(void)
{

}
