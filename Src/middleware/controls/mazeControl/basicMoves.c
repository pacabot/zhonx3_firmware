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

#define UTURN_OFFSET 10

static int moveHalfCell_IN(float max_speed, float end_speed);
static int moveHalfCell_OUT(float max_speed, float end_speed);
static int moveMainDist(float max_speed, float end_speed);
static int moveOffsetDist(double offset, float max_speed, float end_speed);
static int rotate180WithCal(enum rotationTypeEnum rotation_type, char wall_presence, float speed_rotation);
static int rotate90WithCal(enum rotationTypeEnum rotation_type, char wall_presence, float speed_rotation);

/**************************************************************************************/
/***************                    Basic Moves                    ********************/
/**************************************************************************************/

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
    move(0, (OFFSET_DIST * 2.00) + repositionGetFrontDist(), max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :    |    :
 *      :    |    :
 *      o         :
 */
int moveMainDist(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(OFFSET_DIST); //absolute position into a cell
    move(0, MAIN_DIST, max_speed, max_speed); //distance with last move offset

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :         :
 *      :         :
 *      o         :
 */
int moveOffsetDist(double offset, float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
    move(0, (OFFSET_DIST * 2.00) - offset, max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

int rotate180WithCal(enum rotationTypeEnum rotation_type, char wall_presence, float speed_rotation)
{
    // chose the correct turn for re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
    }
    else
    {
        moveRotateInPlaceCW180(speed_rotation, speed_rotation);
    }

    return POSITION_CONTROL_E_SUCCESS;
}

int rotate90WithCal(enum rotationTypeEnum rotation_type, char wall_presence, float speed_rotation)
{
    // chose the correct turn for re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
    }
    else
    {
        moveRotateInPlaceCW180(speed_rotation, speed_rotation);
    }

    return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************************/
/***************                   Avanced Moves                   ********************/
/**************************************************************************************/

/*
 *           |
 *      o    |    o
 *      :    |    :
 *      :    |    :
 *      o         o
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
    moveMainDist(max_speed, max_speed);
    spyPostGetOffset(&offset);

    if (repositionGetFrontDist() > 0.00 || repositionGetFrontDist() < 0.00)
    {
        moveOffsetDist(repositionGetFrontDist(), max_speed, end_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, FRONT OFFSET = %d\n\r", (int32_t)repositionGetFrontDist());
#endif
    }
    else if (offset.left_x != 0)
    {
        moveOffsetDist(offset.left_x, max_speed, end_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, L_OFFSET = %d\n\r", (int32_t)offset.left_x);
#endif
    }
    else if (offset.right_x != 0)
    {
        moveOffsetDist(offset.right_x, max_speed, end_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL, R_OFFSET = %d\n\r", (int32_t)offset.right_x);
#endif
    }
    else
    {
        moveOffsetDist(0.00, max_speed, end_speed);
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf("MOVE CELL\n\r");
#endif
    }
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    |    :
 *      :    |    :
 *      o_________o
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
    moveOffsetDist(repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    if (repositionGetFrontDist() > 0.00 || repositionGetFrontDist() < 0.00)
    {
        bluetoothWaitReady();
        bluetoothPrintf("MOVE START CELL, FRONT OFFSET = %d\n\r", (int32_t)repositionGetFrontDist());
    }
    else
        bluetoothPrintf("MOVE START CELL\n\r");
#endif

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    '._    <<<
 *      :           <<<
 *      o_________o
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
    moveOffsetDist(repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    if (repositionGetFrontDist() > 0.00 || repositionGetFrontDist() < 0.00)
    {
        bluetoothWaitReady();
        bluetoothPrintf("MOVE ROTATE CW90, FRONT OFFSET = %d\n\r", (int32_t)repositionGetFrontDist());
    }
    else
        bluetoothPrintf("MOVE ROTATE CW90\n\r");
#endif

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *  >>>    _.'    :
 *  >>>           :
 *      o_________o
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
    move(0, (OFFSET_DIST * 2.00) + repositionGetFrontDist(), max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    if (repositionGetFrontDist() > 0.00 || repositionGetFrontDist() < 0.00)
    {
        bluetoothWaitReady();
        bluetoothPrintf("MOVE ROTATE CCW90, FRONT OFFSET = %d\n\r", (int32_t)repositionGetFrontDist());
    }
    else
        bluetoothPrintf("MOVE ROTATE CCW90\n\r");
#endif

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :    __
 *      :    |
 *      o_________o
 */
int moveRotateInPlaceCW90(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    move(90, 0, max_speed, max_speed);

#ifdef DEBUG_BASIC_MOVES
        bluetoothPrintf("MOVE ROTATE IN PLACE CW180\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   __
 *      :    |
 *      o_________o
 */
int moveRotateInPlaceCCW90(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    move(-90, 0, max_speed, max_speed);

#ifdef DEBUG_BASIC_MOVES
        bluetoothPrintf("MOVE ROTATE IN PLACE CW180\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   __
 *      :    |
 *      o_________o
 */
int moveRotateInPlaceCW180(float max_speed, float end_speed)
{
    while (hasMoveEnded() != TRUE)
    {
    }
    move(180, 0, max_speed, max_speed);

#ifdef DEBUG_BASIC_MOVES
        bluetoothPrintf("MOVE ROTATE IN PLACE CW180\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/*		 _________
 * 		o	 _	  o
 * 		:   / )   :
 * 		:   \'    :
 * 		o    |    o
 * 			 v
 */
int moveUTurn(float speed_rotation, float max_speed, float end_speed)
{
    char wall_presence = FALSE;
    // memorize wall presence before move HALF CELL IN
    if (getWallPresence(RIGHT_WALL) == TRUE)
    {
        wall_presence = RIGHT_WALL;
    }
    else if (getWallPresence(LEFT_WALL) == TRUE)
    {
        wall_presence = LEFT_WALL;
    }
    // move HALF CELL IN
    while (hasMoveEnded() != TRUE);
    moveHalfCell_IN(max_speed, 0);
    while (hasMoveEnded() != TRUE);

    // chose the correct turn for re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCW90(speed_rotation, speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
        frontCal(speed_rotation);
        while (hasMoveEnded() != TRUE);
        moveRotateInPlaceCCW90(speed_rotation, speed_rotation);
    }
    else
    {
        moveRotateInPlaceCW180(speed_rotation, speed_rotation);
    }

    // go back and go out for maximize correct alignment
    repositionSetInitialPosition(HALF_CELL_LENGTH);
    move(0, -1.00 * (HALF_CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS - UTURN_OFFSET)), max_speed, 0);
    while (hasMoveEnded() != TRUE)
    {
    }
    repositionSetInitialPosition(Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS + UTURN_OFFSET);
    move(0, (HALF_CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS - UTURN_OFFSET)), max_speed, end_speed);
    while (hasMoveEnded() != TRUE)
    {
    }

    moveHalfCell_OUT(max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothPrintf("MOVE U TURN\n\r");
#endif
    return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************************/
/***************                    Tests Moves                    ********************/
/**************************************************************************************/
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
    Vmax = 400;
    Vrotate = 300;

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
    //    moveRotateCW90(Vmin, Vmin);
    //    moveCell(2, Vmax, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);
    moveRotateCW90(Vmin, Vmin);
    moveRotateCCW90(Vmin, Vmin);

    moveRotateCW90(Vmin, Vmin);
    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);

    //    moveRotateCCW90(Vmin, Vmin);
    //    //    moveHalfCell_IN(Vmin, Vmin);
    //    moveCell(2, Vmax, Vmin);
    //    moveRotateCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
    //    moveRotateCW90(Vmin, Vmin);
    //    moveRotateCW90(Vmin, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveRotateCW90(Vmin, Vmin);
    //    moveRotateCCW90(Vmin, Vmin);
    //    moveRotateCW90(Vmin, Vmin);
    //    moveCell(1, Vmax, Vmin);
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

