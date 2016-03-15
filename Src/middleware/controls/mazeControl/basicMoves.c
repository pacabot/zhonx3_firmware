/**************************************************************************/
/*!
    @file    basicMoves.c
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

/* Middleware declarations */
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/controls/mazeControl/reposition.h"

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

/* Declarations for this module */
#include "middleware/controls/mazeControl/basicMoves.h"

/**************************************************************************************/
/***************                    Basic Moves                    ********************/
/**************************************************************************************/

/*
 * 			 :
 * 		o    :    o
 * 		:    :    :
 * 		:    :    :
 * 		o         o
 */
int moveCell(unsigned long nb_cell, float max_speed, float end_speed)
{
	unsigned int i;

	if (nb_cell == 0)
		return POSITION_CONTROL_E_SUCCESS;

	move_params.initial_position = OFFSET_DIST;

	for(i = 0; i < (nb_cell - 1); i++)
	{
		while(hasMoveEnded() != TRUE);
		move(0, (CELL_LENGTH), max_speed, max_speed);
	}
	while(hasMoveEnded() != TRUE);
	move(0, (CELL_LENGTH - (OFFSET_DIST * 2.00)), max_speed, end_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + repositionGetPostDist(-OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 * 		o	 	  o
 * 		:         :
 * 		:    :    :
 * 		o         o
 */
int moveHalfCell_IN(float max_speed, float end_speed)
{
	move_params.initial_position = OFFSET_DIST;

	while(hasMoveEnded() != TRUE);
	move(0, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 * 			 :
 * 		o	 :	  o
 * 		:    :    :
 * 		:         :
 * 		o         :
 */
int moveHalfCell_OUT(float max_speed, float end_speed)
{
	move_params.initial_position = HALF_CELL_LENGTH;

	while(hasMoveEnded() != TRUE);
	move(0, HALF_CELL_LENGTH - OFFSET_DIST, max_speed, end_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + repositionGetPostDist(-OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 * 			 :
 * 		o    :    o
 * 		:    :    :
 * 		:    :    :
 * 		o_________o
 */
int moveStartCell(float max_speed, float end_speed)
{
	move_params.initial_position = Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS;

	while(hasMoveEnded() != TRUE);
	move(0, ((CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS)) - OFFSET_DIST), max_speed, end_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + repositionGetPostDist(-OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 *		 	 :
 * 		o	 :	  o
 * 		:    '._    <<<
 * 		:			<<<
 * 		o_________o
 */
int moveRotateCW90(float max_speed, float end_speed)
{
	move_params.initial_position = (CELL_LENGTH - OFFSET_DIST); //ignore rotate

	while(hasMoveEnded() != TRUE);
	move(90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + repositionGetPostDist(-OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 *		 	 :
 * 		o	 :	  o
 * 	>>>    _.'	  :
 * 	>>>	          :
 * 		o_________o
 */
int moveRotateCCW90(float max_speed, float end_speed)
{
	move_params.initial_position = (CELL_LENGTH - OFFSET_DIST); //ignore rotate

	while(hasMoveEnded() != TRUE);
	move(-90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + repositionGetPostDist(-OFFSET_DIST), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************************/
/***************                Combination Moves                  ********************/
/**************************************************************************************/

/*		 _________
 * 		o	 _	  o
 * 		:   / )   :
 * 		:   \'    :
 * 		o    :    o
 * 			 v
 */
int moveUTurn(float speed_rotation, float max_speed, float end_speed)
{
	while(hasMoveEnded() != TRUE);
	moveHalfCell_IN(max_speed, 0);

	//frontCal(speed_rotation);

	while(hasMoveEnded() != TRUE);
	if (getWallPresence(RIGHT_WALL) == WALL_PRESENCE)
	{
		move (90, 0, speed_rotation, speed_rotation);
		frontCal(speed_rotation);
		while(hasMoveEnded() != TRUE);
		move (90, 0, speed_rotation, speed_rotation);
	}
	else if (getWallPresence(LEFT_WALL) == WALL_PRESENCE)
	{
		move (-90, 0, speed_rotation, speed_rotation);
		frontCal(speed_rotation);
		while(hasMoveEnded() != TRUE);
		move (-90, 0, speed_rotation, speed_rotation);
	}
	else
	{
		move (-180, 0, speed_rotation, speed_rotation);
	}

	moveHalfCell_OUT(max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int rotate180WithCal(enum rotationTypeEnum rotation_type, float max_speed, float end_speed)
{
	frontCal(max_speed);

	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, 0);
	else
		move(-90, 0, max_speed, 0);
	while(hasMoveEnded() != TRUE);
	/***************************************/

	frontCal(max_speed);

	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, end_speed);
	else
		move(-90, 0, max_speed, end_speed);
	while(hasMoveEnded() != TRUE);
	/***************************************/

	return POSITION_CONTROL_E_SUCCESS;
}

int rotate90WithCal(enum rotationTypeEnum rotation_type, float max_speed, float end_speed)
{
	/***************************************/
	if (rotation_type == CW)
		move(90, 0, max_speed, 0);
	else
		move(-90, 0, max_speed, 0);
	while(hasMoveEnded() != TRUE);
	/***************************************/

	frontCal(max_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

void mainControlDisplayTest(void)
{
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen(MAIN_AREA);
		ssd1306PrintIntAtLine(0, 0,  "speed dist =  ",(int) (speedControlGetCurrentDist() * 100), &Font_5x8);
		ssd1306PrintIntAtLine(0, 1, "follow err =  ",(int) (wallFollowGetCommand()), &Font_5x8);
		ssd1306PrintIntAtLine(0, 2, "right_dist =  ",(int) (positionControlHasMoveEnded()), &Font_5x8);
		ssd1306PrintIntAtLine(0, 3, "gyro =  ",(int16_t) gyroGetAngle(), &Font_5x8);
		//		ssd1306PrintIntAtLine(0, 4, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		//		ssd1306PrintIntAtLine(0, 0, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		ssd1306Refresh();
	}
	telemetersStop();
	motorsDriverSleep(ON);
}

void movesTest()
{
	mainControlInit();
	telemetersStart();

	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(WALL_FOLLOW);

	HAL_Delay(2000);

	int Vmin, Vmax, Vrotate;
	Vmin = 400;
	Vmax = 400;
	Vrotate = 400;

	moveStartCell(Vmax, Vmax);
	moveCell(1, Vmax, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveCell(2, Vmax, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveCell(2, Vmax, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveCell(1, Vmax, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveCell(1, Vmax, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveRotateCCW90(Vmin, Vmin);
	moveRotateCW90(Vmin, Vmin);
	moveCell(1, Vmax, Vmin);
	//	moveCell(5, Vmax, Vmin);
	//	moveRotateCW90(Vmin, Vmin);
	//	moveCell(2, Vmax, Vmin);
	//	moveUTurn(Vrotate, Vmax, Vmin);
	//	moveRotateCCW90(Vmin, Vmin);
	//	moveCell(2, Vmax, Vmin);
	//	moveRotateCW90(Vmin, Vmin);
	//	moveCell(2, Vmax, Vmin);
	//	moveRotateCCW90(Vmin, Vmin);
	//	moveCell(1, Vmax, Vmin);
	//	moveRotateCW90(Vmin, 0);

	telemetersStop();
	mainControlDisplayTest();
}

void rotateTest()
{
	mainControlInit();
	telemetersStart();

	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(NO_FOLLOW);

	HAL_Delay(2000);

	moveUTurn(100, 100, 100);
	return;
	move(-90, 0, 8, 8); //rotation example

	while(hasMoveEnded() != TRUE){
		while(expanderJoyFiltered()!=JOY_LEFT)
		{
			ssd1306ClearScreen(MAIN_AREA);

			ssd1306PrintIntAtLine(0, 0,  "L_DIST_REL =  ",(signed int) encoderGetDist(ENCODER_L), &Font_5x8);
			ssd1306PrintIntAtLine(0, 1, "R_DIST_REL =  ",(signed int) encoderGetDist(ENCODER_R), &Font_5x8);

			ssd1306Refresh();
			HAL_Delay(100);
		}
	}

	mainControlDisplayTest();
}

