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

	speed_params.initial_speed = 0;

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlLoop(void)
{
	speedControlLoop();
	positionControlLoop();
	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

int move(float angle, float radius_or_distance, float max_speed, float end_speed)//enum speedRate speed_rate, float end_speed_ratio)
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

	speed_params.sign = SIGN(radius_or_distance);
	radius_or_distance = fabsf(radius_or_distance);

	position_params.sign = SIGN(angle);
	angle = fabsf(angle);

	/* Apply the correction factor, delete function with the future gyro compensation */
	slip_compensation = 1.09;

	speed_params.end_speed  = end_speed;
	speed_params.max_speed 	= max_speed;
	speed_params.accel 		= max_speed * 2;//(MAX_ACCEL)
	speed_params.decel 		= max_speed * 2;//

	if (angle == 0)
	{
		distance = radius_or_distance;

		speedProfileCompute(distance);
		positionProfileCompute(0,0);
	}
	else
	{
		distance_per_wheel = (2.0 * PI * ROTATION_DIAMETER * (angle / 360.0)) * slip_compensation;
		distance = fabsf((PI * (2 * radius_or_distance) * (angle / 360.0)));

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

	move(0, 90, 200, 200);
	while(speed_control.end_control != 1);
	move(90, 90, 200, 200);
	while(speed_control.end_control != 1);
	move(-90, 90, 200, 200);
	while(speed_control.end_control != 1);
	move(0, 360, 1000, 200);
	while(speed_control.end_control != 1);
	move(90, 90, 200, 200);
	while(speed_control.end_control != 1);
	move(0, 90, 200, 0);
	while(speed_control.end_control != 1);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "speed dist =  ",(int) (speed_control.current_distance * 100), &Font_5x8);
		ssd1306PrintInt(10,  15, "posit.dist =  ",(int) (position_control.end_control * 100), &Font_5x8);
		ssd1306PrintInt(10,  25, "right_dist =  ",(int) (position_control.end_control * 100), &Font_5x8);
		ssd1306PrintInt(10,  35, "error =  ",(int16_t) speed_control.speed_error, &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		ssd1306Refresh();
	}
	antiBounceJoystick();
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

