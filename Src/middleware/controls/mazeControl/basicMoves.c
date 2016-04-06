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
#include "middleware/controls/mazeControl/spyPost.h"

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
 * 			 |
 * 		o    |    o
 * 		:    |    :
 * 		:    |    :
 * 		o         o
 */
int moveCell(unsigned long nb_cell, float max_speed, float end_speed)
{
    unsigned int i;
    spyPostGetOffsetsStruct offset;

    if (nb_cell == 0)
        return POSITION_CONTROL_E_SUCCESS;

    for (i = 0; i < (nb_cell - 1); i++)
    {
        while (hasMoveEnded() != TRUE);
        repositionSetInitialPosition(OFFSET_DIST); //absolute position into a cell
        move(0, (CELL_LENGTH), max_speed, max_speed);
    }
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(OFFSET_DIST); //absolute position into a cell
    move(0, MAIN_DIST, max_speed, end_speed); //distance with last move offset
    spyPostGetOffset(&offset);
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(CELL_LENGTH - OFFSET_DIST);	//absolute position into a cell
    if (repositionGetFrontDist() > 0.00 || repositionGetFrontDist() < 0.00)
    {
        move(0, (OFFSET_DIST * 2.00) - repositionGetFrontDist(), max_speed, end_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, FRONT OFFSET = %d\n\r", (int32_t)repositionGetFrontDist());
#endif
    }
    else if (offset.left_x != 0)
    {
        move(0, (OFFSET_DIST * 2.00) - offset.left_x, max_speed, max_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, L_OFFSET = %d\n\r", (int32_t)offset.left_x);
#endif
    }
    else
    {
        move(0, (OFFSET_DIST * 2.00) - offset.right_x, max_speed, max_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, R_OFFSET = %d\n\r", (int32_t)offset.right_x);
#endif
    }
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 * 		o	 	  o
 * 		:         :
 * 		:    |    :
 * 		o         o
 */
int moveHalfCell_IN(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition(OFFSET_DIST);
    move(0, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 * 			 |
 * 		o	 |	  o
 * 		:    |    :
 * 		:         :
 * 		o         :
 */
int moveHalfCell_OUT(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(HALF_CELL_LENGTH);
    move(0, HALF_CELL_LENGTH - OFFSET_DIST, max_speed, end_speed);
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(CELL_LENGTH - (OFFSET_DIST));
    move(0, (OFFSET_DIST * 2.00) - repositionGetFrontDist(), max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 * 			 |
 * 		o    |    o
 * 		:    |    :
 * 		:    |    :
 * 		o_________o
 */
int moveStartCell(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition(Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS);
    move(0, ((CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS)) - OFFSET_DIST), max_speed, end_speed);
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition(CELL_LENGTH - (OFFSET_DIST));
    move(0, (OFFSET_DIST * 2.00) - repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothPrintf("MOVE START CELL\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *		 	 |
 * 		o	 |	  o
 * 		:    '._    <<<
 * 		:			<<<
 * 		o_________o
 */
int moveRotateCW90(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    move(90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition((CELL_LENGTH - OFFSET_DIST));
    move(0, (OFFSET_DIST * 2.00) - repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothPrintf("MOVE ROTATE CW90\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *		 	 |
 * 		o	 |	  o
 * 	>>>    _.'	  :
 * 	>>>	          :
 * 		o_________o
 */
int moveRotateCCW90(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    move(-90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition((CELL_LENGTH - OFFSET_DIST));
    move(0, (OFFSET_DIST * 2.00) - repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothPrintf("MOVE ROTATE CCW90 = OFFSET_DIST * 2 \n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************************/
/***************                   Avanced Moves                   ********************/
/**************************************************************************************/
/*		 _________
 * 		o	 _	  o
 * 		:   / )   :
 * 		:   \'    :
 * 		o    |    o
 * 			 v
 */
int moveUTurn(float speed_rotation, float max_speed, float end_speed)
{
    char wall_presence = 0xFF;
    if (getWallPresence(RIGHT_WALL) == WALL_PRESENCE)
    {
        wall_presence = RIGHT_WALL;
    }
    else if (getWallPresence(LEFT_WALL) == WALL_PRESENCE)
    {
        wall_presence = LEFT_WALL;
    }
    while (hasMoveEnded() != TRUE);
    moveHalfCell_IN(max_speed, 0);
    while (hasMoveEnded() != TRUE);
    if (wall_presence == RIGHT_WALL)
    {
        move(90, 0, speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        move(90, 0, speed_rotation, speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        move(-90, 0, speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        move(-90, 0, speed_rotation, speed_rotation);
    }
    else
    {
        move(-180, 0, speed_rotation, speed_rotation);
    }

    moveHalfCell_OUT(max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothPrintf("MOVE U TURN\n\r");
#endif
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
    while (hasMoveEnded() != TRUE)
    {
    }
    /***************************************/

    frontCal(max_speed);

    /***************************************/
    if (rotation_type == CW)
        move(90, 0, max_speed, end_speed);
    else
        move(-90, 0, max_speed, end_speed);
    while (hasMoveEnded() != TRUE)
    {
    }
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
    while (hasMoveEnded() != TRUE)
    {
    }
    /***************************************/

    frontCal(max_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

void mainControlDisplayTest(void)
{
    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintIntAtLine(0, 0, "speed dist =  ", (int) (speedControlGetCurrentDist() * 100), &Font_5x8);
        ssd1306PrintIntAtLine(0, 1, "follow err =  ", (int) (wallFollowGetCommand()), &Font_5x8);
        ssd1306PrintIntAtLine(0, 2, "right_dist =  ", (int) (positionControlHasMoveEnded()), &Font_5x8);
        ssd1306PrintIntAtLine(0, 3, "gyro =  ", (int16_t) gyroGetAngle(), &Font_5x8);
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
    Vmin = 300;
    Vmax = 300;
    Vrotate = 100;

    //   moveStartCell(Vmax, Vmax);
    //    moveCell(1, Vmax, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);

    //    ssd1306Refresh();
    //    telemetersStop();
    //    motorsDriverSleep(ON);
    //    while(1);
    moveStartCell(Vmax, Vmax);
    moveCell(1, Vmax, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveCell(2, Vmax, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    //    moveHalfCell_IN(Vmin, Vmin);
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

    while (hasMoveEnded() != TRUE)
    {
        while (expanderJoyFiltered() != JOY_LEFT)
        {
            ssd1306ClearScreen(MAIN_AREA);

            ssd1306PrintIntAtLine(0, 0, "L_DIST_REL =  ", (signed int) encoderGetDist(ENCODER_L), &Font_5x8);
            ssd1306PrintIntAtLine(0, 1, "R_DIST_REL =  ", (signed int) encoderGetDist(ENCODER_R), &Font_5x8);

            ssd1306Refresh();
            HAL_Delay(100);
        }
    }

    mainControlDisplayTest();
}

