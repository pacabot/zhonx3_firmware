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
#include "peripherals/telemeters/telemeters.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/mainControl.h"

int mainControlInit(void)
{
	motorsInit();
	encodersInit();
	mulimeterInit();
	//	telemetersInit();
	speedControlInit();
	positionControlInit();
	transfertFunctionInit();

	speed_params.end_speed_ratio = 0;

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlLoop(void)
{
	speedControlLoop();
	positionControlLoop();
	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

int move(float angle, float radius_or_distance, enum speedRate speed_rate, float end_speed_ratio)
{
	pid_loop.start_state = FALSE;
	speedControlInit();
	positionControlInit();
	transfertFunctionInit();

	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);

	float distance;
	float slip_compensation;
	float distance_per_wheel;
	const float ROTATION_DIAMETER = sqrt(pow(WHEELS_DISTANCE, 2) + pow(WHEELS_SPACING, 2));

	/* Apply the correction factor, delete function with the future gyro compensation */
	switch (speed_rate) {

	case LOWSPEED:
		slip_compensation = 1.09;
		break;

	case MEDIUMSPEED :
		slip_compensation = 1.01;
		break;

	case FASTSPEED:
		slip_compensation = 1.02;
		break;

	case HIGHSPEED:
		slip_compensation = 1.035;
		break;
	}

	speed_params.max_speed 	= (MAX_SPEED * speed_rate)/100.0;
	speed_params.accel 		= (MAX_ACCEL * speed_rate)/100.0;
	speed_params.decel 		= (MAX_DECEL * speed_rate)/100.0;

	if (angle == 0)
	{
		distance = radius_or_distance;

		speed_params.end_speed_ratio = end_speed_ratio;

		speedProfileCompute(distance);
		positionProfileCompute(0,0);
	}
	else
	{
		distance_per_wheel = (2.0 * PI * ROTATION_DIAMETER * (angle / 360.0)) * slip_compensation;
		distance = (PI * (2 * radius_or_distance) * (angle / 360.0));

		speed_params.end_speed_ratio = end_speed_ratio;

		positionProfileCompute(distance_per_wheel, speedProfileCompute(distance));
	}

	motorsSleepDriver(OFF);
	pid_loop.start_state = TRUE;
	return POSITION_CONTROL_E_SUCCESS;
}

void mainControlTest(void)
{
	mainControlInit();
	HAL_Delay(500);
	move(90, 0, LOWSPEED, 100);
	while(speed_control.end_control != 1);
//	move(0, 10, LOWSPEED, 100);
	while(speed_control.end_control != 1);
//	move(0, 10, LOWSPEED, 100);
//	while(speed_control.end_control != 1);
//	move(90, 40, LOWSPEED, 100);
//	while(speed_control.end_control != 1);
//	move(90, 40, LOWSPEED, 0);
//	while(speed_control.end_control != 1);
//	while(position_control.end_control != 1);
//	motorsSleepDriver(ON);
//	move(0, 100, LOWSPEED, 100);
//	while(speed_control.end_control != 1);
//	move(180, 90, LOWSPEED, 100);
//	while(position_control.end_control != 1);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "speed dist =  ",(int) (speed_control.current_distance * 100), &Font_5x8);
		ssd1306PrintInt(10,  15, "posit.dist =  ",(int) (position_control.end_control * 100), &Font_5x8);
		ssd1306PrintInt(10,  25, "right_dist =  ",(int16_t) encoderGetDistance(&right_encoder), &Font_5x8);
		ssd1306PrintInt(10,  35, "error =  ",(int16_t) speed_control.speed_error, &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		ssd1306Refresh();
	}
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

