/**************************************************************************/
/*!
    @file    mainControl.c
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
#include <middleware/controls/motionControl/transfertFunction.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/mainControl.h"

 speed_params_struct speed_params;

int mainControlInit(void)
{
	motorsInit();
	encodersInit();
	mulimeterInit();
//	telemetersInit();
	speedControlInit();
	positionControlInit();
	transfertFunctionInit();

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlLoop(void)
{
	speedControlLoop();
	positionControlLoop();
	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

/**************************************************************************/
/*!
    ***BASICS FORMULAS***
    	   _____
    	  /  d
    t =  /  ---
        V	Acc

		 V²
	d = ---
	    Acc

	d = V * t

	V = Acc * t
 */
/**************************************************************************/
void straightMove(float distance, enum speedRate speed_rate)
{
	float acceleration_distance;
	float deceleration_distance;

	speed_params.max_speed 	= (MAX_SPEED  * speed_rate)/10.0; //
	speed_params.accel 		= (MAX_ACCEL * speed_rate)/10.0;
	speed_params.decel 		= (MAX_DECEL * speed_rate)/10.0;

	acceleration_distance = pow(speed_params.max_speed, 2)/speed_params.accel;
	deceleration_distance = pow(speed_params.max_speed, 2)/speed_params.decel;

	if ((acceleration_distance + acceleration_distance) <= distance)
	{
		speed_params.accel_dist = acceleration_distance;
		speed_params.decel_dist = deceleration_distance;
		speed_params.maintain_dist = (distance - (acceleration_distance + acceleration_distance));
	}
	else
	{
		float clipping_ratio;

		clipping_ratio =  (distance / (acceleration_distance + deceleration_distance));

		speed_params.accel_dist = (acceleration_distance * clipping_ratio);
		speed_params.decel_dist = (deceleration_distance * clipping_ratio);
		speed_params.maintain_dist = 0;

	}

	speed_params.distance_consign = distance;

	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);
	motorsSleepDriver(OFF);
	pid_loop.start_state = TRUE;
}

void mainControlTest(void)
{
	mainControlInit();
	HAL_Delay(1000);
	straightMove(300, LOWSPEED);

	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "dist_c =  ",(int) (speed_control.current_distance), &Font_5x8);
		ssd1306PrintInt(10,  15, "acc =  ",(int16_t) speed_params.accel_dist, &Font_5x8);
		ssd1306PrintInt(10,  25, "dcc =  ",(int16_t) speed_params.decel_dist, &Font_5x8);
		ssd1306PrintInt(10,  35, "error =  ",(int16_t) speed_control.speed_error, &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		ssd1306Refresh();
	}
	antiBounceJoystick();
	pid_loop.start_state = FALSE;
	motorsSleepDriver(ON);
}

