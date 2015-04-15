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
speed_params_struct speed_params;
speed_control_struct speed_control;
pid_control_struct speed_control_pid;
arm_pid_instance_f32 encoder_pid_instance;

int speedControlInit(void)
{
	encoder_pid_instance.Kp = 500.0;
	encoder_pid_instance.Ki = 0;
	encoder_pid_instance.Kd = 800;

	speed_control.current_distance = 0;
	speed_control.gap_distance_per_loop = 0;
	speed_control.current_distance_consign = 0;
	speed_control.old_distance = 0;
	speed_control.current_speed = 0;
	speed_control.end_control = 0;

	speed_control.speed_error = 0;
	speed_control.speed_command = 0;
	speed_control.speed_consign = 0;

	speed_params.final_speed = 0;
	speed_params.initial_speed = 0;

	speed_control.speed_pid.instance = &encoder_pid_instance;

	pidControllerInit(speed_control.speed_pid.instance);


	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControlLoop(void)
{
	speed_control.current_distance = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder)) / 2;
	//	speedCompute();

	if (speed_params.nb_loop_accel > 0)
	{
		speed_params.nb_loop_accel--;
		speed_control.speed_consign += speed_params.accel_dist_per_loop;
		speed_control.current_distance_consign += speed_control.speed_consign;
	}
	else if (speed_params.nb_loop_maint > 0)
	{
		speed_params.nb_loop_maint--;
		speed_control.current_distance_consign += speed_control.speed_consign;
	}
	else if (speed_params.nb_loop_decel > 0)
	{
		speed_params.nb_loop_decel--;
		speed_control.speed_consign -= speed_params.decel_dist_per_loop;
		speed_control.current_distance_consign += speed_control.speed_consign;
	}
	else if (speed_params.nb_loop_decel <= 0)
	{
		speed_control.current_distance_consign = speed_params.distance_consign;
		speed_control.end_control = 1;
	}

	speed_control.speed_error = speed_control.current_distance_consign - speed_control.current_distance;		//for distance control
	speed_control.speed_command = pidController(speed_control.speed_pid.instance, speed_control.speed_error);

	speed_control.old_distance = speed_control.current_distance;

	return SPEED_CONTROL_E_SUCCESS;
}

int speedCompute(void)
{
	speed_control.gap_distance_per_loop = speed_control.current_distance - speed_control.old_distance;	//delta distance per loop
	speed_control.current_speed = (speed_control.gap_distance_per_loop * HI_TIME_FREQ);			//actual speed (mm/s)

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
float speedProfileCompute(float distance)
{
	if (distance == 0)
		return 0.0;

	speed_params.accel_dist = 0.5f * (speed_params.max_speed + speed_params.initial_speed) * ((speed_params.max_speed - speed_params.initial_speed)/speed_params.accel);
	speed_params.decel_dist = pow((speed_params.max_speed * ((100 - speed_params.end_speed_ratio) / 100.0f)), 2) / (2 * speed_params.decel);

	speed_params.accel_dist_per_loop = speed_params.accel / pow(HI_TIME_FREQ, 2);
	speed_params.decel_dist_per_loop = speed_params.decel / pow(HI_TIME_FREQ, 2);

	speed_control.speed_consign = (speed_params.initial_speed / HI_TIME_FREQ);
	speed_control.current_distance_consign = 0;
	speed_control.end_control = 0;


	if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
	{
		float clipping_ratio;
		clipping_ratio =  (distance / (speed_params.accel_dist + speed_params.decel_dist));
		speed_params.accel_dist *= clipping_ratio;
		speed_params.decel_dist *= clipping_ratio;
	}

	speed_params.nb_loop_accel = ((sqrt((speed_params.initial_speed * speed_params.initial_speed) +
			2.0f * speed_params.accel * speed_params.accel_dist) -
			speed_params.initial_speed) / ( speed_params.accel)) * HI_TIME_FREQ;

	speed_params.max_speed = speed_params.initial_speed + (speed_params.accel * (speed_params.nb_loop_accel/HI_TIME_FREQ));

	speed_params.nb_loop_decel = ((sqrt((speed_params.max_speed * speed_params.max_speed) -
			2.0f * speed_params.decel * speed_params.decel_dist) -
			speed_params.max_speed) / ( -1.0f * speed_params.decel)) * HI_TIME_FREQ;

	if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
	{
		speed_params.maintain_dist = 0;
		speed_params.nb_loop_maint = 0;
	}
	else
	{
		speed_params.maintain_dist = distance - (speed_params.accel_dist + speed_params.decel_dist);
		speed_params.nb_loop_maint = ((speed_params.maintain_dist / speed_params.max_speed) * HI_TIME_FREQ);
	}

	if (speed_params.end_speed_ratio <= 0)
	{
		speed_params.initial_speed = 0;
	}
	else
	{
		speed_params.initial_speed = speed_params.initial_speed + (((speed_params.nb_loop_accel / HI_TIME_FREQ) * speed_params.accel) -
				((speed_params.nb_loop_decel / HI_TIME_FREQ) * speed_params.decel));
	}

	speed_params.distance_consign = distance;

	return ((speed_params.nb_loop_accel + speed_params.nb_loop_decel + speed_params.nb_loop_maint)) / HI_TIME_FREQ;
}
