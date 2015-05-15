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
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/times_base/times_base.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/followControl.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/mainControl.h"

control_params_struct control_params;

double ROTATION_DIAMETER = sqrt(pow(WHEELS_DISTANCE, 2) + pow(WHEELS_SPACING, 2));

int mainControlInit(void)
{
	motorsInit();
	encodersInit();
	mulimeterInit();
	telemetersInit();
	speedControlInit();
	positionControlInit();
	followControlInit();
	transfertFunctionInit();
	adxrs620Init();

	speed_params.initial_speed = 0;

	control_params.follow_state = 0;
	control_params.position_state = 0;
	control_params.speed_state = 0;

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlLoop(void)
{
	if (control_params.follow_state == TRUE)
	{
		followControlLoop();
//		control_params.position_state = FALSE;
	}
	else
	{
		control_params.position_state = TRUE;
	}
	if (control_params.speed_state == TRUE)
		speedControlLoop();
	if (control_params.position_state == TRUE)
		positionControlLoop();

	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

int move(float angle, float radius_or_distance, float max_speed, float end_speed)
{
	pid_loop.start_state = FALSE;

	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);
	GyroResetAngle();

	float distance;
	float slip_compensation;
	float distance_per_wheel;

	speed_params.sign = SIGN(radius_or_distance);
	radius_or_distance = fabsf(radius_or_distance);

	position_params.sign = SIGN(angle);
	angle = fabsf(angle);

	/* Apply the correction factor, delete function with the future gyro compensation */
	slip_compensation = 1.0;

	speed_params.end_speed  = end_speed;
	speed_params.max_speed 	= max_speed;
	speed_params.accel 		= MAX_ACCEL;
	speed_params.decel 		= MAX_ACCEL;

	control_params.speed_state = TRUE;

	if (angle == 0)
	{
		control_params.follow_state = TRUE;
		position_control.position_type = ENCODERS;
//		if (follow_control.follow_type != NOFOLLOW)
//			control_params.follow_state = TRUE;
//		else
//			control_params.follow_state = FALSE;


		speedProfileCompute(radius_or_distance);
		positionProfileCompute(0,0);
	}
	else
	{
		position_control.position_type = GYRO;
		control_params.follow_state = FALSE;

		distance_per_wheel = (2.00 * PI * ROTATION_DIAMETER * (angle / 360.00)) * slip_compensation;
		distance = fabsf((PI * (2.00 * radius_or_distance) * (angle / 360.00)));

		positionProfileCompute(distance_per_wheel, speedProfileCompute(distance));
	}

	pid_loop.start_state = TRUE;
	return POSITION_CONTROL_E_SUCCESS;
}

char isEndMove(void)
{
	if (position_control.end_control == TRUE && speed_control.end_control == TRUE)
		return TRUE;
	else
		return FALSE;
}

int frontCal(float max_speed)
{
	int i = 0;
	char save_folow_type = follow_control.follow_type;
	telemetersDistancesTypeDef distances;
	float relative_dist = 0.0;

		follow_control.follow_type = ALIGN_FRONT;
		move(0, 0, 0, 0);
		while (follow_control.succes != TRUE)
		{
			if (timeOut(1, i) == TRUE)
				return POSITION_CONTROL_E_ERROR;
			i++;
		}

	/**************************************************************************/

	getTelemetersDistance(&distances);
	relative_dist = (distances.distance_front_left + distances.distance_front_right) / 2;
	if (relative_dist < MAX_DIST_FOR_ALIGN)
	{
		move(0, relative_dist - CENTER_DISTANCE, max_speed, 0);
		while(isEndMove() != TRUE);
	}

	follow_control.follow_type = save_folow_type;

	return POSITION_CONTROL_E_SUCCESS;
}

int rotate180WithCal(enum rotation_type_enum rotation_type, float max_speed, float end_speed)
{
	frontCal(max_speed);

	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, 0);
	else
		move(-90, 0, max_speed, 0);
	while(isEndMove() != TRUE);
	/***************************************/

	frontCal(max_speed);

	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, end_speed);
	else
		move(-90, 0, max_speed, end_speed);
	while(isEndMove() != TRUE);
	/***************************************/

	return POSITION_CONTROL_E_SUCCESS;
}

int rotate90WithCal(enum rotation_type_enum rotation_type, float max_speed, float end_speed)
{
	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, 0);
	else
		move(-90, 0, max_speed, 0);
	while(isEndMove() != TRUE);
	/***************************************/

	frontCal(max_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveCell(unsigned long nb_cell, float max_speed, float end_speed)
{
	if (nb_cell == 0)
		return POSITION_CONTROL_E_SUCCESS;

	telemetersDistancesTypeDef distances;
	float relative_dist = 0.0;

	while(isEndMove() != TRUE);

	if (nb_cell > 1)
	{
		move(0, (CELL_LENGTH * (nb_cell - 1)), max_speed, max_speed);
		while(isEndMove() != TRUE);
	}
	move(0, (NO_FOLLOW_DIST - STRAIGHT_DIST), max_speed, max_speed);
	while(isEndMove() != TRUE);
	control_params.follow_state = FALSE;

	move(0, (CELL_LENGTH - STRAIGHT_DIST - NO_FOLLOW_DIST), max_speed, max_speed);
	while(isEndMove() != TRUE);
	control_params.follow_state = TRUE;
	move(0, 2.00 * STRAIGHT_DIST, max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveHalfCell(float max_speed, float end_speed)
{
	while(isEndMove() != TRUE);
	move(0, 60, max_speed, max_speed);
	while(isEndMove() != TRUE);
	move(0, (1.00 * STRAIGHT_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveEndCell(float max_speed, float end_speed)
{
	while(isEndMove() != TRUE);
	move(0, 60, max_speed, max_speed);
	while(isEndMove() != TRUE);
//	move(0, (.00 * STRAIGHT_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveStartCell(float max_speed, float end_speed)
{
	while(isEndMove() != TRUE);
	move(0, ((CELL_LENGTH) - ((STRAIGHT_DIST) + 39.00)), max_speed, max_speed);
	while(isEndMove() != TRUE);
	move(0, (2.00 * STRAIGHT_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveRotateCW90(float max_speed, float end_speed)
{
	while(isEndMove() != TRUE);
	move(90, ((CELL_LENGTH / 2.00) - (STRAIGHT_DIST)), max_speed, max_speed); //
	while(isEndMove() != TRUE);
	control_params.follow_state = TRUE;
	move(0, (STRAIGHT_DIST * 2.00), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveRotateCCW90(float max_speed, float end_speed)
{
	while(isEndMove() != TRUE);
	move(-90, ((CELL_LENGTH / 2.00) - (STRAIGHT_DIST)), max_speed, max_speed); //
	while(isEndMove() != TRUE);
	control_params.follow_state = TRUE;
	move(0, (STRAIGHT_DIST * 2.00), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int moveUTurn(float speed_rotation, float max_speed, float end_speed)
{
//	char save_folow_type = follow_control.follow_type;
	while(isEndMove() != TRUE);
//	follow_control.follow_type = ALIGN_FRONT;
	moveEndCell(max_speed, 0);
	while(isEndMove() != TRUE);
	move (180, 0, speed_rotation, 0);
	while(isEndMove() != TRUE);
//	control_params.follow_state = TRUE;
//	follow_control.follow_type = save_folow_type;
	moveHalfCell(max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

void mainControlDisplayTest(void)
{
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "speed dist =  ",(int) (speed_control.current_distance * 100), &Font_5x8);
		ssd1306PrintInt(10,  15, "follow err =  ",(int) (follow_control.follow_error), &Font_5x8);
		ssd1306PrintInt(10,  25, "right_dist =  ",(int) (position_control.end_control * 100), &Font_5x8);
		ssd1306PrintInt(10,  35, "gyro =  ",(int16_t) GyroGetAngle(), &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		//		bluetoothPrintf("pwm right :%d \t %d \n",(int)transfert_function.right_motor_pwm, (int)(follow_control.follow_error*100));
		//		bluetoothPrintInt("error", follow_control.follow_error);
		//		transfert_function.right_motor_pwm = (speed_control.speed_command - (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;
		//		transfert_function.left_motor_pwm  = (speed_control.speed_command + (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;

		ssd1306Refresh();
	}
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

void mainControlTest(void)
{
	mainControlInit();
	telemetersStart();
	HAL_Delay(500);

	follow_control.follow_type = NOFOLLOW;

	rotate180WithCal(CCW, 400, 0);

	telemetersStop();
	mainControlDisplayTest();
}

void followWallTest()
{
	mainControlInit();
	telemetersStart();
	motorsSleepDriver(OFF);
	HAL_Delay(500);
	move(0, 0, 0, 0);

	control_params.follow_state = TRUE;
	follow_control.follow_type = FOLLOW_WALL;//NOFOLLOW

	moveUTurn(300, 300, 0);
while(1);
	move(0, 0, 0, 0);
	moveStartCell(50, 10);
	moveRotateCW90(50, 10);
	moveRotateCW90(50, 10);
	moveRotateCCW90(50, 10);
	moveRotateCCW90(50, 10);

	while(1);

	moveRotateCW90(600, 200);
	moveRotateCW90(600, 200);
	moveCell(1, 600, 200);
	moveRotateCCW90(600, 200);
	moveRotateCCW90(600, 200);
	HAL_Delay(1000);
	moveCell(4, 1000, 200);
	HAL_Delay(1000);
	moveRotateCW90(600, 200);
	moveRotateCCW90(600, 200);
	moveRotateCW90(600, 200);
	moveRotateCCW90(600, 200);
	moveRotateCW90(600, 200);
	moveRotateCCW90(600, 200);
	moveRotateCW90(600, 200);
	moveCell(1, 600, 200);

	telemetersStop();
	mainControlDisplayTest();
	mainControlDisplayTest();
}

void followLineTest()
{
	mainControlDisplayTest();
}

void rotateTest()
{
	mainControlInit();
	HAL_Delay(500);

	follow_control.follow_type = NOFOLLOW;
	position_control.position_type = GYRO;

	move(90, 90, 300, 0);
	//	while(isEndMove() != TRUE);
	//	move(0, 180, 100, 0);
	//	while(isEndMove() != TRUE);
	//	move(0, 0, 0, 0);
	//	while(isEndMove() != TRUE);

	mainControlDisplayTest();
}

void curveRotateTest()
{
	mainControlDisplayTest();
}

