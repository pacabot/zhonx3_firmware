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
#include "middleware/controls/motionControl/lineFollowControl.h"
#include <middleware/controls/motionControl/wallFollowControl.h>

/* Declarations for this module */
#include "middleware/controls/motionControl/mainControl.h"

typedef struct
{
	char wall_follow_state;
	char line_follow_state;
	char position_state;
	char speed_state;
}control_params_struct;

typedef struct
{
	enum mainControlWallFollowType moveType;
	walls cellState;
	double initial_position;
}move_params_struct;

static control_params_struct control_params;
static move_params_struct move_params;

int debug_1 = 0;

double ROTATION_DIAMETER;

int mainControlInit(void)
{
	ROTATION_DIAMETER = sqrt(pow(WHEELS_DISTANCE, 2) + pow(WHEELS_SPACING, 2));

	motorsInit();
	encodersInit();
	mulimeterInit();
	telemetersInit();
	speedControlInit();
	positionControlInit();
	wallFollowControlInit();
	lineFollowControlInit();
	transfertFunctionInit();
	adxrs620Init();
	positionControlSetPositionType(ENCODERS);
	pid_loop.start_state = TRUE;
	mainControlSetFollowType(FALSE);

	move_params.moveType = STRAIGHT;

	control_params.wall_follow_state = 0;
	control_params.line_follow_state = 0;

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControl_IT(void)
{
    if (pid_loop.start_state == FALSE)
    {
        return MAIN_CONTROL_E_SUCCESS;
    }

	if (control_params.line_follow_state == TRUE)
	{
		lineFollowControlLoop();
		control_params.wall_follow_state = FALSE;
	}
	else if (control_params.wall_follow_state == TRUE)
	{
		wallFollowControlLoop();
	}

	positionControlLoop();
	speedControlLoop();
	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlSetFollowType(enum mainControlFollowType follow_type)
{
	switch(follow_type)
	{
	case LINE_FOLLOW:
		control_params.wall_follow_state = FALSE;
		control_params.line_follow_state = TRUE;
		return MAIN_CONTROL_E_SUCCESS;
	case WALL_FOLLOW:
		control_params.line_follow_state = FALSE;
		control_params.wall_follow_state = TRUE;
		return MAIN_CONTROL_E_SUCCESS;
	case NO_FOLLOW:
		control_params.line_follow_state = FALSE;
		control_params.wall_follow_state = FALSE;
		return MAIN_CONTROL_E_SUCCESS;
	}

	return MAIN_CONTROL_E_ERROR;
}

enum mainControlFollowType mainControlGetFollowType()
{
	if (control_params.wall_follow_state == TRUE)
		return WALL_FOLLOW;
	else if (control_params.line_follow_state == TRUE)
		return LINE_FOLLOW;
	else
		return NO_FOLLOW;
}

enum mainControlWallFollowType mainControlGetWallFollowType()
{
	return move_params.moveType;
}

int move(double angle, double radius_or_distance, double max_speed, double end_speed)
{
	//	pid_loop.start_state = FALSE; //todo optimize

	encodersReset();
	gyroResetAngle();

	float distance;
	float slip_compensation;
	float distance_per_wheel;

	speedControlSetSign((double)SIGN(radius_or_distance));
	radius_or_distance = fabsf(radius_or_distance);

	positionControlSetSign((double)SIGN(angle));
	angle = fabsl(angle);

	/* Apply the correction factor, delete function with the future gyro compensation */
	slip_compensation = 1; //0.9;

	if (lround(angle) == 0)
	{
		move_params.moveType = STRAIGHT;

		speedProfileCompute(radius_or_distance, max_speed, end_speed);
		positionProfileCompute(0, 0, max_speed);
	}
	else
	{
		move_params.moveType = CURVE;

		distance_per_wheel = (2.00 * PI * ROTATION_DIAMETER * (angle / 360.00)) * slip_compensation;
		distance = fabsf((PI * (2.00 * radius_or_distance) * (angle / 360.00)));

		positionProfileCompute(distance_per_wheel, speedProfileCompute(distance, max_speed, end_speed), max_speed);
	}

	//	pid_loop.start_state = TRUE;
	return POSITION_CONTROL_E_SUCCESS;
}

char hasMoveEnded(void)
{
	if (positionControlHasMoveEnded() == TRUE && speedControlHasMoveEnded() == TRUE)
		return TRUE;
	else
		return FALSE;
}

double mouveGetInitialPosition(void)
{
	return move_params.initial_position;
}

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
	move(0, (OFFSET_DIST * 2.00) + speedMaintainCompute(), max_speed, end_speed);

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
	move(0, (OFFSET_DIST * 2.00) + speedMaintainCompute(), max_speed, end_speed);

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
	move_params.initial_position = Z3_CENTER_BACK_DIST;

	while(hasMoveEnded() != TRUE);
	move(0, ((CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS)) - OFFSET_DIST), max_speed, end_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00) + speedMaintainCompute(), max_speed, end_speed);

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
	move(0, (OFFSET_DIST * 2.00), max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

/*
 *		 	 :
 * 		o	 :	  o
 * 	>>>    _·'	  :
 * 	>>>	          :
 * 		o_________o
 */
int moveRotateCCW90(float max_speed, float end_speed)
{
	move_params.initial_position = (CELL_LENGTH - OFFSET_DIST); //ignore rotate

	while(hasMoveEnded() != TRUE);
	move(-90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
	while(hasMoveEnded() != TRUE);
	move(0, (OFFSET_DIST * 2.00), max_speed, end_speed);

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

	while(hasMoveEnded() != TRUE);
	move (180, 0, speed_rotation, 0);

	moveHalfCell_OUT(max_speed, end_speed);

	return POSITION_CONTROL_E_SUCCESS;
}

int frontCal(float max_speed)
{
	int i = 0;
	float relative_dist = 0.0;

	move(0, 0, 0, 0);
//	while (wall_follow_control.succes != TRUE)
//	{
//		if (timeOut(1, i) == TRUE)
//			return POSITION_CONTROL_E_ERROR;
//		i++;
//	}

	/**************************************************************************/

	relative_dist = (getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2;
//	if (relative_dist < MAX_DIST_FOR_ALIGN)
//	{
//		move(0, relative_dist - CENTER_DISTANCE, max_speed, 0);
//		while(hasMoveEnded() != TRUE);
//	}


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
		ssd1306PrintInt(10,  5,  "speed dist =  ",(int) (speedControlGetCurrentDist() * 100), &Font_5x8);
		ssd1306PrintInt(10,  15, "follow err =  ",(int) (wallFollowGetCommand()), &Font_5x8);
		ssd1306PrintInt(10,  25, "right_dist =  ",(int) (positionControlHasMoveEnded()), &Font_5x8);
		ssd1306PrintInt(10,  35, "gyro =  ",(int16_t) gyroGetAngle(), &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		//		bluetoothPrintf("pwm right :%d \t %d \n",(int)transfert_function.right_motor_pwm, (int)(follow_control.follow_error*100));
		//		bluetoothPrintInt("error", follow_control.follow_error);
		//		transfert_function.right_motor_pwm = (speed_control.speed_command - (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;
		//		transfert_function.left_motor_pwm  = (speed_control.speed_command + (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;

		ssd1306Refresh(MAIN_AREA);
	}
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);
}

void mainControlTest(void)
{
	mainControlInit();
	telemetersStart();
	HAL_Delay(500);

	rotate180WithCal(CCW, 400, 0);

	telemetersStop();
	mainControlDisplayTest();
}

void followWallTest()
{
	ledPowerBlink(0, 0, 0);
	mainControlInit();
	telemetersStart();

	positionControlSetPositionType(GYRO);
	control_params.wall_follow_state = TRUE;

	//	setCellState();
	HAL_Delay(2000);
	move(0, 0, 0, 0);
	motorsDriverSleep(OFF);

	expanderLedState(1,0);
	expanderLedState(2,0);
	expanderLedState(3,0);

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
	//	telemetersStart();
	motorsDriverSleep(OFF);

	//	position_control.position_type = GYRO;
	control_params.wall_follow_state = TRUE;

	HAL_Delay(2000);
	move(0, 0, 0, 0);
	moveRotateCCW90(100, 100);

	//	while(1);
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

