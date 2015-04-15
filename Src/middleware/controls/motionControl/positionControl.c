/**************************************************************************/
/*!
    @file    positionControl.c
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

/* Declarations for this module */
#include "middleware/controls/motionControl/positionControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
position_control_struct position_control;
position_params_struct position_params;
arm_pid_instance_f32 telemeters_pid_instance;

int positionControlInit(void)
{
	telemeters_pid_instance.Kp = 500;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 20;

	position_control.current_angle = 0;
	position_control.position_command = 0;
	position_control.position_error = 0;
	position_control.current_diff_dist = 0;
	position_control.current_diff_dist_consign = 0;
	position_control.position_consign = 0;
	position_control.position_pid.instance = &telemeters_pid_instance;
	position_control.end_control = 0;

	pidControllerInit(position_control.position_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int positionControlLoop(void)
{
	position_control.current_diff_dist = (encoderGetDistance(&left_encoder) - encoderGetDistance(&right_encoder));

	if (position_params.nb_loop_accel > 0)
	{
		position_params.nb_loop_accel--;
		position_control.position_consign += position_params.accel_dist_per_loop;
		position_control.current_diff_dist_consign += position_control.position_consign;
	}
	else if (position_params.nb_loop_maint > 0)
	{
		position_params.nb_loop_maint--;
		position_control.current_diff_dist_consign += position_control.position_consign;
	}
	else if (position_params.nb_loop_decel > 0)
	{
		position_params.nb_loop_decel--;
		position_control.position_consign -= position_params.decel_dist_per_loop;
		position_control.current_diff_dist_consign += position_control.position_consign;
	}
	else if (position_params.nb_loop_decel <= 0)
	{
		position_control.current_diff_dist_consign = position_params.distance_consign;
		position_control.end_control = 1;
	}

	position_control.position_error = position_control.current_diff_dist_consign - position_control.current_diff_dist;		//for distance control
	position_control.position_command = pidController(position_control.position_pid.instance, position_control.position_error);

	position_control.old_distance = position_control.current_diff_dist;

	return SPEED_CONTROL_E_SUCCESS;
}

/**************************************************************************/
/*!
 ***BASICS FORMULAS***
    	   	   _____
    	  	  / 2.d
    	t =  /  ---
        	V	Acc

		  	  V²
		d = -----
	    	2.Acc

		 	 1
		d = --- Acc x t²
		 	 2

			   2.d
		Acc = -----
		 	    t²

		d = V x t

		V = Vi + (Acc x t)

		       --------------,
    	  	  / Vi² + 2.Acc.d - Vi
    	t =  V____________________
        			  Acc

        	   -2(t.Vi-d)
        Acc = ------------
                   t²

                2(t.Vi-d)
        Dcc = ------------
                   t²
 */
/**************************************************************************/
float positionProfileCompute(float distance, float time)
{
	if (time == 0.0f)
	{
		time = 1;
//		speed_params.accel_dist = pow(position_params.max_speed, 2) / (2 * position_params.decel);
//		speed_params.decel_dist = pow(position_params.max_speed, 2) / (2 * position_params.decel);
	}
//	else
//	{
		time /= 2.0f;

		position_params.accel_dist = 0.5f * distance;
		position_params.decel_dist = 0.5f * distance;

		position_params.accel = 2.0f * (distance / 2.0f) / (pow(time, 2));

		position_params.decel = position_params.accel;
//	}

	position_params.accel_dist_per_loop = position_params.accel / pow(HI_TIME_FREQ, 2);
	position_params.decel_dist_per_loop = position_params.decel / pow(HI_TIME_FREQ, 2);

	position_params.nb_loop_accel = (sqrt((2 * position_params.accel_dist) / position_params.accel) * HI_TIME_FREQ);
	position_params.nb_loop_decel = (sqrt((2 * position_params.decel_dist) / position_params.decel) * HI_TIME_FREQ);

	position_params.distance_consign = distance;

	return (position_params.nb_loop_accel + position_params.nb_loop_decel + position_params.nb_loop_maint);
}
