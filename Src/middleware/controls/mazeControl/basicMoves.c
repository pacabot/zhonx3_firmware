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
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/wall_sensors/wall_sensors.h"

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

/* Declarations for this module */
#include "middleware/controls/mazeControl/basicMoves.h"

#define UTURN_OFFSET 25

typedef struct
{
    repositionGetOffsetsStruct frontCal;
    spyPostGetOffsetsStruct spyPost;
} getOffsetsStruct;

static int moveHalfCell_IN(double max_speed, double end_speed);
static int moveHalfCell_OUT(double max_speed, double end_speed);
static int moveMainDist(getOffsetsStruct *offset, double max_speed, double end_speed);
static int moveOffsetDist(getOffsetsStruct *offset, double max_speed, double end_speed);
static int moveRotateInPlaceCW90(double speed_rotation);
static int moveRotateInPlaceCCW90(double speed_rotation);
static int moveRotateInPlaceCW180(double speed_rotation);
static int frontAlignment(float max_speed);

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
int moveHalfCell_IN(double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(OFFSET_DIST);
    move(0, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 * 		o	 	  o
 * 		:    |    :
 * 		:         :
 * 		o         :
 */
int moveHalfCell_OUT(double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(HALF_CELL_LENGTH);
    move(0, HALF_CELL_LENGTH - OFFSET_DIST, max_speed, end_speed);
    while (hasMoveEnded() != TRUE);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :    |    :
 *      :    |    :
 *      o         :
 */
int moveMainDist(getOffsetsStruct *offset, double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(OFFSET_DIST); //absolute position into a cell
    move(0, MAIN_DIST, max_speed, max_speed); //distance with last move offset
    spyPostGetOffset(&offset->spyPost);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :         :
 *      :         :
 *      o         :
 */
int moveOffsetDist(getOffsetsStruct *offset, double max_speed, double end_speed)
{
    double offset_error = 0;

    if (offset->spyPost.left_x != 0)
    {
        offset_error = (double)offset->spyPost.left_x;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(" L_OFFSET = %d, TYPE = %d\n", (int32_t)offset->spyPost.left_x, offset->spyPost.left_spyPostType);
#endif
    }
    else if (offset->spyPost.right_x != 0)
    {
        offset_error = (double)offset->spyPost.right_x;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(" R_OFFSET = %d, TYPE = %d\n", (int32_t)offset->spyPost.right_x, offset->spyPost.right_spyPostType);
#endif
    }
    else if (offset->frontCal.front_dist != 0)
    {
        offset_error = (double)offset->frontCal.front_dist;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(" F_OFFSET = %d\n", (int32_t)offset->frontCal.front_dist);
#endif
    }
    else
    {
        offset_error = 0.00;
    }
    if (fabs(offset_error) > MAX_FRONT_DIST_ERROR - 1) // -1 for non null move
    {
        if(offset_error < 0.00)
        {
            offset_error = -MAX_FRONT_DIST_ERROR;
        }
        else
        {
            offset_error = MAX_FRONT_DIST_ERROR;
        }
    }
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
    move(0, (OFFSET_DIST * 2.00) + offset_error, max_speed, end_speed);
    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   _|__
 *      :    !
 *      o_________o
 */
int moveRotateInPlaceCW90(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    move(90, 0, speed_rotation, 0);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :  __|_
 *      :    !
 *      o_________o
 */
int moveRotateInPlaceCCW90(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    move(-90, 0, speed_rotation, 0);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *       _________
 *      o         o
 *      :   .->
 *      :   '-<
 *      o_________o
 */
int moveRotateInPlaceCW180(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    move(180, 0, speed_rotation, 0);

    return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************************/
/***************                  Specials Moves                   ********************/
/**************************************************************************************/
int frontAlignment(float max_speed)
{
    double relative_dist = 0.00;
    while (hasMoveEnded() != TRUE);
    if (getWallPresence(FRONT_WALL) == WALL_PRESENCE)
    {
        if (getTelemeterDist(TELEMETER_FR) > getTelemeterDist(TELEMETER_FL))
        {
            move(-30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FR) - getTelemeterDist(TELEMETER_FL))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    move(30, 0, max_speed, max_speed);
                    return 0xFF;
                }
            }
        }
        else
        {
            move(30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FL) - getTelemeterDist(TELEMETER_FR))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    move(-30, 0, max_speed, max_speed);
                    return 0xFF;
                }
            }
        }
        relative_dist = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) - 21.00;
        move(0, relative_dist, 100, 100);
    }
    return 0;
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
int moveCell(unsigned int nb_cell, double max_speed, double end_speed)
{
    unsigned int i;
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE %d CELL,", nb_cell);
#endif

    if (nb_cell == 0)
        return POSITION_CONTROL_E_SUCCESS;

    for (i = 0; i < (nb_cell - 1); i++)
    {
        moveMainDist(&offset, max_speed, max_speed);
        repositionGetFrontDist(&offset.frontCal);
        moveOffsetDist(&offset, max_speed, end_speed);
    }
    moveMainDist(&offset, max_speed, max_speed);
    repositionGetFrontDist(&offset.frontCal);
    moveOffsetDist(&offset, max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    |    :
 *      :    |    :
 *      o_________o
 */
int moveStartCell(double max_speed, double end_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE START CELL");
#endif
    while (hasMoveEnded() != TRUE);
    repositionSetInitialPosition(Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS);
    move(0, ((CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS)) - OFFSET_DIST), max_speed, end_speed);
    while (hasMoveEnded() != TRUE);

    repositionGetFrontDist(&offset.frontCal);
    moveOffsetDist(&offset, max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    '._    <<<
 *      :           <<<
 *      o_________o
 */
int moveRotateCW90(double max_speed, double end_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE ROTATE CW90");
#endif
    while (hasMoveEnded() != TRUE);
    move(90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
    while (hasMoveEnded() != TRUE);

    repositionGetFrontDist(&offset.frontCal);
    moveOffsetDist(&offset, max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *  >>>    _.'    :
 *  >>>           :
 *      o_________o
 */
int moveRotateCCW90(double max_speed, double end_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE ROTATE CCW90");
#endif
    while (hasMoveEnded() != TRUE);
    move(-90, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, max_speed);
    while (hasMoveEnded() != TRUE);

    repositionGetFrontDist(&offset.frontCal);
    moveOffsetDist(&offset, max_speed, end_speed);

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *       _________
 *      o         o
 *      :   .->
 *      :   '-<
 *      o_________o
 */
int moveRotateInPlace180WithCal(wallSelectorEnum wall_presence, double speed_rotation)
{
    // chose the correct turn for re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        moveRotateInPlaceWithCalCW90(wall_presence, speed_rotation);
        moveRotateInPlaceCW90(speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        moveRotateInPlaceWithCalCCW90(wall_presence, speed_rotation);
        moveRotateInPlaceCCW90(speed_rotation);
    }
    else
    {
        moveRotateInPlaceCW180(speed_rotation);
    }

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   _|__
 *      :    !
 *      o_________o
 */
int moveRotateInPlaceWithCalCW90(wallSelectorEnum wall_presence, double speed_rotation)
{
    moveRotateInPlaceCW90(speed_rotation);
    // chose re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        frontAlignment(speed_rotation);
    }

    return POSITION_CONTROL_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :  __|_
 *      :    !
 *      o_________o
 */
int moveRotateInPlaceWithCalCCW90(wallSelectorEnum wall_presence, double speed_rotation)
{
    moveRotateInPlaceCCW90(speed_rotation);
    // chose re-calibrate the robot if possible
    if (wall_presence == LEFT_WALL)
    {
        frontAlignment(speed_rotation);
    }

    return POSITION_CONTROL_E_SUCCESS;
}

/*		 _________
 * 		o	 _	  o
 * 		:   / )   :
 * 		:   \'    :
 * 		o    |    o
 * 			 v
 */
int moveUTurn(double speed_rotation, double max_speed, double end_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
    wallSelectorEnum wall_presence = FALSE;
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
    moveHalfCell_IN(max_speed, 0);
    // chose the correct turn for re-calibrate the robot if possible
    moveRotateInPlace180WithCal(wall_presence, speed_rotation);
    while (hasMoveEnded() != TRUE);
    // go back and go out for maximize correct alignment
    mainControlSetFollowType(NO_FOLLOW); //todo this is the shit
    positionControlSetPositionType(ENCODERS);
    repositionSetInitialPosition(HALF_CELL_LENGTH);
    move(0, -1.00 * (HALF_CELL_LENGTH - Z3_CENTER_BACK_DIST - UTURN_OFFSET + 15), max_speed, 0);
    mainControlSetFollowType(WALL_FOLLOW); //todo this is the shit
    positionControlSetPositionType(GYRO);
    repositionSetInitialPosition(Z3_CENTER_BACK_DIST + UTURN_OFFSET);
    while (hasMoveEnded() != TRUE);
    move(0, (HALF_CELL_LENGTH - Z3_CENTER_BACK_DIST - UTURN_OFFSET), max_speed, max_speed);

    moveHalfCell_OUT(max_speed, max_speed);

    repositionGetFrontDist(&offset.frontCal);
    moveOffsetDist(&offset, max_speed, end_speed);

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE U TURN");
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
    Vmin = 100;
    Vmax = 100;
    Vrotate = 100;

    //test Uturn
    //moveStartCell(Vmax, Vmax);
    //moveUTurn(Vrotate, Vmax, Vmax);
    //return;

    double abs_encoders = encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R);
    //maze
//    moveStartCell(Vmax, Vmax);
    moveCell(1, Vmax, Vmin);
//    moveRotateCW90(Vrotate, Vrotate);
//    moveCell(2, Vmax, Vmin);
//    moveRotateCW90(Vrotate, Vrotate);
//    moveRotateCCW90(Vrotate, Vrotate);
//    moveRotateCCW90(Vrotate, Vrotate);
    while (hasMoveEnded() != TRUE);
    abs_encoders = (encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R)) - abs_encoders;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintIntAtLine(0, 1, "abs  dist =  ", (int)abs_encoders / 2, &Font_5x8);
    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", 239 + 179 + 109 + 179 + 179 + 109 + 109 + 109, &Font_5x8);
    ssd1306Refresh();

    while(1);

    moveCell(2, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(3, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveUTurn(Vrotate, Vmax, Vmin);
    moveCell(1, Vmax, Vmin);
    moveRotateCCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(2, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(2, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveUTurn(Vrotate, Vmax, Vmin);
    moveRotateCCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);

    return;

    // zigzag
    moveStartCell(Vmax, Vmax);
    moveCell(1, Vmax, Vmin);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveRotateCW90(Vrotate, Vrotate);
    moveRotateCCW90(Vrotate, Vrotate);
    moveCell(1, Vmax, Vmin);

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

