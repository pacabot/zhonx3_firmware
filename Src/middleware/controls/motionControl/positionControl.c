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
	telemeters_pid_instance.Kp = 300;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 800;

	position_control.current_angle = 0;
	position_control.position_command = 0;
	position_control.position_error = 0;
	position_control.current_diff_dist = 0;
	position_control.current_diff_dist_consign = 0;
	position_control.position_consign = 0;
	position_control.position_pid.instance = &telemeters_pid_instance;
	position_control.end_control = 0;

	position_params.sign = 1;

	pidControllerInit(position_control.position_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int positionControlLoop(void)
{
	if (position_params.sign > 0)
		position_control.current_diff_dist = encoderGetDistance(&left_encoder) - encoderGetDistance(&right_encoder);
	else
		position_control.current_diff_dist = encoderGetDistance(&right_encoder) - encoderGetDistance(&left_encoder);

	if (position_params.nb_loop_accel > 0)
	{
		position_params.nb_loop_accel--;
		position_control.position_consign += position_params.accel_dist_per_loop;
		position_control.current_diff_dist_consign += position_control.position_consign;
	}
	else if (position_control.current_diff_dist < (position_params.accel_dist + position_params.maintain_dist))
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
	position_control.position_command = (pidController(position_control.position_pid.instance, position_control.position_error)) * (float)position_params.sign;

	position_control.old_distance = position_control.current_diff_dist;

	return SPEED_CONTROL_E_SUCCESS;
}

/**************************************************************************/
/*!
 ***BASICS FORMULAS***

		      ___   _________
			 / 2 x / Acc x d
		t = v_____v__________	//without initial speed
				  Acc

			        __________________
			- Vi + / Vi²+ 2 x Acc x d
		t = ______v___________________	//with initial speed
				      Acc

		  	  V²
		d = -----
	    	2.Acc

		 	 1
		d = --- Acc x t²
		 	 2

			 	      1
		d = Vi x t + --- Acc x t²
		 	 	      2

		Vf = Vi + Acc x t 	//instantaneous speed

	        Vi + Vf
		V = -------			//average speed
		       2

			   2.d
		Acc = -----
		 	    t²

		d = V x t

		V = Vi + (Acc x t)
										  ________________________
			 / 1 \             / 1 \	 /
		v =  |---| x t x Acc + |---| x  V t² x Acc² - 4 x Acc x d	//v = f(t,d,Acc)
			 \ 2 /			   \ 2 /

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
	if (distance == 0)
	{
		position_params.nb_loop_accel = 0;
		position_params.nb_loop_decel = 0;
		position_params.nb_loop_maint = 0;
		return (0);
	}
	if (time == 0.0f)
	{
		time = 1;
		//		speed_params.accel_dist = pow(position_params.max_speed, 2) / (2 * position_params.decel);
		//		speed_params.decel_dist = pow(position_params.max_speed, 2) / (2 * position_params.decel);
	}
	else
	{
		position_params.accel = MAX_TURN_ACCEL;
		position_params.decel = MAX_TURN_ACCEL;

		position_params.max_speed = (0.5 * time * position_params.accel - 0.5 *
				sqrt((time * time) * (position_params.accel * position_params.accel) - 4.0 * distance * position_params.accel));


		position_params.accel_dist = pow(position_params.max_speed, 2)/(2.0 * position_params.accel);
		position_params.decel_dist = pow(position_params.max_speed, 2)/(2.0 * position_params.decel);

		if ((position_params.accel_dist + position_params.decel_dist ) > distance)
		{
			double clipping_ratio;
			clipping_ratio =  (distance / (position_params.accel_dist + position_params.decel_dist));
			position_params.accel_dist *= clipping_ratio;
			position_params.decel_dist *= clipping_ratio;

			position_params.accel *= (1.0+clipping_ratio);
			position_params.decel *= (1.0+clipping_ratio);

//			position_params.max_speed = (sqrt(2.0) * sqrt(0.5 * distance * position_params.accel_dist));
//
//			position_params.accel_dist = pow(position_params.max_speed, 2)/(2.0 * position_params.accel);
//			position_params.decel_dist = pow(position_params.max_speed, 2)/(2.0 * position_params.decel);
		}

		position_params.maintain_dist = distance - (position_params.accel_dist + position_params.decel_dist);

		position_params.accel_time = ((sqrt(2.0) * sqrt(position_params.accel * position_params.accel_dist)) / position_params.accel);
		position_params.decel_time = ((sqrt(2.0) * sqrt(position_params.decel * position_params.decel_dist)) / position_params.decel);



		position_params.accel_speed_avrg = position_params.accel_dist / position_params.accel_time;
		position_params.decel_speed_avrg = position_params.decel_dist / position_params.decel_time;

		position_params.speed_average = (position_params.accel_dist + position_params.decel_dist + position_params.maintain_dist) /
				(position_params.accel_time + position_params.decel_time + (position_params.maintain_dist /position_params.max_speed));
	}

	position_params.accel_dist_per_loop = position_params.accel / pow(HI_TIME_FREQ, 2);
	position_params.decel_dist_per_loop = position_params.decel / pow(HI_TIME_FREQ, 2);

	position_params.nb_loop_accel = position_params.accel_time * HI_TIME_FREQ;
	position_params.nb_loop_decel = position_params.decel_time * HI_TIME_FREQ;
	position_params.nb_loop_maint = (position_params.maintain_dist / position_params.max_speed) * HI_TIME_FREQ;

	position_params.distance_consign = distance;

	return (position_params.nb_loop_accel + position_params.nb_loop_decel + position_params.nb_loop_maint);
}
