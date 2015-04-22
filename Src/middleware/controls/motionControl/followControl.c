/**************************************************************************/
/*!
    @file    followControl.c
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
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/positionControl.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/followControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
position_control_struct position_control;
position_params_struct position_params;
arm_pid_instance_f32 telemeters_pid_instance;

int followControlInit(void)
{
	telemeters_pid_instance.Kp = 300;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 800;

	follow_control.follow_pid.instance = &telemeters_pid_instance;

	pidControllerInit(position_control.position_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int followControlLoop(void)
{
//	if (position_params.sign > 0)
//		position_control.current_diff_dist = encoderGetDistance(&left_encoder) - encoderGetDistance(&right_encoder);
//	else
//		position_control.current_diff_dist = encoderGetDistance(&right_encoder) - encoderGetDistance(&left_encoder);
//
//	if (position_params.nb_loop_accel > 0)
//	{
//		position_params.nb_loop_accel--;
//		position_control.position_consign += position_params.accel_dist_per_loop;
//		position_control.current_diff_dist_consign += position_control.position_consign;
//	}
//	else if (position_control.current_diff_dist < (position_params.accel_dist + position_params.maintain_dist))
//	{
//		position_params.nb_loop_maint--;
//		position_control.current_diff_dist_consign += position_control.position_consign;
//	}
//	else if (position_params.nb_loop_decel > 0)
//	{
//		position_params.nb_loop_decel--;
//		position_control.position_consign -= position_params.decel_dist_per_loop;
//		position_control.current_diff_dist_consign += position_control.position_consign;
//	}
//	else if (position_params.nb_loop_decel <= 0)
//	{
//		position_control.current_diff_dist_consign = position_params.distance_consign;
//		position_control.end_control = 1;
//	}
//
//	position_control.position_error = position_control.current_diff_dist_consign - position_control.current_diff_dist;		//for distance control
//	position_control.position_command = (pidController(position_control.position_pid.instance, position_control.position_error)) * (float)position_params.sign;
//
//	position_control.old_distance = position_control.current_diff_dist;

	return SPEED_CONTROL_E_SUCCESS;
}

