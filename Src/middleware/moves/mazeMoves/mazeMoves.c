/**************************************************************************/
/*!
 @file    mazeMoves.c
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
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/moves/mazeMoves/spyPost.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/moves/basicMoves/basicMoves.h"

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
#include "peripherals/tone/tone.h"

/* Declarations for this module */
#include "middleware/moves/mazeMoves/mazeMoves.h"

#define UTURN_OFFSET 40 //distance between the wall and rear robot in Uturn mode

typedef struct
{
    spyWallGetOffsetsStruct frontCal;
    spyPostGetOffsetsStruct spyPost;
} getOffsetsStruct;

static int mazeMoveHalfCell_IN(double max_speed, double out_speed);
static int mazeMoveHalfCell_OUT(double max_speed, double out_speed);
static int mazeMoveMainDist(getOffsetsStruct *offset, double max_speed, double out_speed);
static int mazeMoveOffsetDist(getOffsetsStruct *offset, double max_speed);
static int mazeMoveRotateInPlaceCW90(double speed_rotation);
static int mazeMoveRotateInPlaceCCW90(double speed_rotation);
static int mazeMoveRotateInPlaceCW180(double speed_rotation);
static int mazeMoveFrontAlignment(float max_speed);

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
int mazeMoveHalfCell_IN(double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(OFFSET_DIST);
    basicMove(0, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, end_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 * 		o	 	  o
 * 		:    |    :
 * 		:         :
 * 		o         :
 */
int mazeMoveHalfCell_OUT(double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(HALF_CELL_LENGTH);
    basicMove(0, HALF_CELL_LENGTH - OFFSET_DIST, max_speed, end_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :    |    :
 *      :    |    :
 *      o         :
 */
int mazeMoveMainDist(getOffsetsStruct *offset, double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(OFFSET_DIST); //absolute position into a cell
    basicMove(0, MAIN_DIST, max_speed, end_speed); //distance with last move offset
    spyPostGetOffset(&offset->spyPost);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :         :
 *      :         :
 *      o         :
 */
int mazeMoveOffsetDist(getOffsetsStruct *offset, double max_speed)
{
    double offset_error = 0;

    if (offset->spyPost.left_x != 0)
    {
        offset_error = (double)offset->spyPost.left_x;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(", L_OFFSET = %d, TYPE = %d", (int32_t)offset->spyPost.left_x, offset->spyPost.left_spyPostType);
#endif
        toneItMode(C4, 300);
    }
    else if (offset->spyPost.right_x != 0)
    {
        offset_error = (double)offset->spyPost.right_x;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(", R_OFFSET = %d, TYPE = %d", (int32_t)offset->spyPost.right_x, offset->spyPost.right_spyPostType);
#endif
        toneItMode(C4, 300);
    }
    else if (offset->frontCal.front_dist != 0)
    {
        offset_error = (double)offset->frontCal.front_dist;
#ifdef DEBUG_BASIC_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(", F_OFFSET = %d", (int32_t)offset->frontCal.front_dist);
#endif
        toneItMode(A5, 200);
    }
    else
    {
        offset_error = 0.00;
    }
    if (offset_error <= (OFFSET_DIST * -2.00))
    {
        while (hasMoveEnded() != TRUE);
        wallFollowSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
        return MAZE_MOVES_E_SUCCESS;
    }
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
    basicMove(0, (OFFSET_DIST * 2.00) + offset_error, max_speed, max_speed);
    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   _|__
 *      :    !
 *      o_________o
 */
int mazeMoveRotateInPlaceCW90(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    basicMove(90, 0, speed_rotation, 0);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :  __|_
 *      :    !
 *      o_________o
 */
int mazeMoveRotateInPlaceCCW90(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    basicMove(-90, 0, speed_rotation, 0);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *       _________
 *      o         o
 *      :   .->
 *      :   '-<
 *      o_________o
 */
int mazeMoveRotateInPlaceCW180(double speed_rotation)
{
    while (hasMoveEnded() != TRUE);
    basicMove(180, 0, speed_rotation, 0);

    return MAZE_MOVES_E_SUCCESS;
}

/**************************************************************************************/
/***************                  Specials Moves                   ********************/
/**************************************************************************************/
int mazeMoveFrontAlignment(float max_speed)
{
    double relative_dist = 0.00;
    while (hasMoveEnded() != TRUE);
    if (getWallPresence(FRONT_WALL) == TRUE)
    {
        if (getTelemeterDist(TELEMETER_FR) > getTelemeterDist(TELEMETER_FL))
        {
            basicMove(-30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FR) - getTelemeterDist(TELEMETER_FL))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    basicMove(30, 0, max_speed, max_speed);
                    return MAZE_MOVES_E_SUCCESS;
                }
            }
        }
        else
        {
            basicMove(30, 0, max_speed, max_speed);
            while (((getTelemeterDist(TELEMETER_FL) - getTelemeterDist(TELEMETER_FR))) > 1.00)
            {
                if (hasMoveEnded() == TRUE)
                {
                    basicMove(-30, 0, max_speed, max_speed);
                    return MAZE_MOVES_E_SUCCESS;
                }
            }
        }
        basicMove(0, 0, 0, 0);
        relative_dist = ((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) - 21.00;
        basicMove(0, relative_dist, 100, 100);
    }
    return MAZE_MOVES_E_SUCCESS;
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
int mazeMoveCell(unsigned int nb_cell, double max_speed, double out_speed)
{
    unsigned int i;
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));

    if (nb_cell < 1)
        return MAZE_MOVES_E_ERROR;

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE %d CELL", nb_cell);
#endif

    if (nb_cell == 0)
        return MAZE_MOVES_E_SUCCESS;

    for (i = 0; i < (nb_cell - 1); i++)
    {
        mazeMoveMainDist(&offset, max_speed, max_speed);
        spyWallGetFrontDist(&offset.frontCal);
        mazeMoveOffsetDist(&offset, max_speed);
    }
    mazeMoveMainDist(&offset, max_speed, out_speed);
    spyWallGetFrontDist(&offset.frontCal);
    mazeMoveOffsetDist(&offset, out_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    |    :
 *      :    |    :
 *      o_________o
 */
int mazeMoveStartCell(double max_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE START CELL");
#endif
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS);
    basicMove(0, ((CELL_LENGTH - (Z3_CENTER_BACK_DIST + HALF_WALL_THICKNESS)) - OFFSET_DIST), max_speed, out_speed);
    while (hasMoveEnded() != TRUE);

    spyWallGetFrontDist(&offset.frontCal);
    mazeMoveOffsetDist(&offset, out_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *      :    '._    <<<
 *      :           <<<
 *      o_________o
 */
int mazeMoveRotateCW90(double max_turn_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE ROTATE CW90");
#endif
    while (hasMoveEnded() != TRUE);
    basicMove(90, (HALF_CELL_LENGTH - OFFSET_DIST), max_turn_speed, max_turn_speed);
    while (hasMoveEnded() != TRUE);

    spyWallGetFrontDist(&offset.frontCal);
    mazeMoveOffsetDist(&offset, out_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *           |
 *      o    |    o
 *  >>>    _.'    :
 *  >>>           :
 *      o_________o
 */
int mazeMoveRotateCCW90(double max_turn_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE ROTATE CCW90");
#endif
    while (hasMoveEnded() != TRUE);
    basicMove(-90, (HALF_CELL_LENGTH - OFFSET_DIST), max_turn_speed, max_turn_speed);
    while (hasMoveEnded() != TRUE);

    spyWallGetFrontDist(&offset.frontCal);
    mazeMoveOffsetDist(&offset, out_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *       _________
 *      o         o
 *      :   .->
 *      :   '-<
 *      o_________o
 */
int mazeMoveRotateInPlace180WithCal(wallSelectorEnum wall_presence, double speed_rotation)
{
    // chose the correct turn for re-calibrate the robot if possible
    if (wall_presence == RIGHT_WALL)
    {
        mazeMoveRotateInPlaceWithCalCW90(speed_rotation);
        mazeMoveRotateInPlaceCW90(speed_rotation);
    }
    else if (wall_presence == LEFT_WALL)
    {
        mazeMoveRotateInPlaceWithCalCCW90(speed_rotation);
        mazeMoveRotateInPlaceCCW90(speed_rotation);
    }
    else
    {
        mazeMoveRotateInPlaceCW180(speed_rotation);
    }

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :   _|__
 *      :    !
 *      o_________o
 */
int mazeMoveRotateInPlaceWithCalCW90(double speed_rotation)
{
    mazeMoveRotateInPlaceCW90(speed_rotation);
    // chose re-calibrate the robot if possible
    mazeMoveFrontAlignment(speed_rotation);

    return MAZE_MOVES_E_SUCCESS;
}

/*
 *
 *      o         o
 *      :  __|_
 *      :    !
 *      o_________o
 */
int mazeMoveRotateInPlaceWithCalCCW90(double speed_rotation)
{
    mazeMoveRotateInPlaceCCW90(speed_rotation);
    // chose re-calibrate the robot if possible
    mazeMoveFrontAlignment(speed_rotation);

    return MAZE_MOVES_E_SUCCESS;
}

/*		 _________
 * 		o	 _	  o
 * 		:   / )   :
 * 		:   \'    :
 * 		o    |    o
 * 			 v
 */
int mazeMoveUTurn(double speed_rotation, double max_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
    wallSelectorEnum wall_presence = FALSE;

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE U TURN");
#endif

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
    mazeMoveHalfCell_IN(max_speed, 0);
    mainControlSetFollowType(NO_FOLLOW); //todo this is the shit
    // chose the correct turn for re-calibrate the robot if possible
    mazeMoveRotateInPlace180WithCal(wall_presence, speed_rotation);//speed_rotation);

    // go back and go out for maximize correct alignment
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(HALF_CELL_LENGTH);
    basicMove(0, -1.00 * (HALF_CELL_LENGTH - Z3_CENTER_BACK_DIST - UTURN_OFFSET + 15), max_speed, 0);

    wallFollowSetInitialPosition(Z3_CENTER_BACK_DIST + UTURN_OFFSET);
    while (hasMoveEnded() != TRUE);
    basicMove(0, (HALF_CELL_LENGTH - Z3_CENTER_BACK_DIST - UTURN_OFFSET), max_speed, max_speed);

    mazeMoveHalfCell_OUT(max_speed, out_speed);
    mainControlSetFollowType(WALL_FOLLOW);
    spyWallGetFrontDist(&offset.frontCal);
    mazeMoveOffsetDist(&offset, out_speed);

    return MAZE_MOVES_E_SUCCESS;
}

/*       _________
 *      o    _    o
 *      :   /|)   :
 *      :   \'    :
 *      o    |    o
 *           ^
 */
int mazeMoveResetStart(double speed_rotation, double max_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
    wallSelectorEnum wall_presence = FALSE;

#ifdef DEBUG_BASIC_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE RESET START");
#endif

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
    mazeMoveHalfCell_IN(max_speed, 0);
    mainControlSetFollowType(NO_FOLLOW); //todo this is the shit
    // chose the correct turn for re-calibrate the robot if possible
    mazeMoveRotateInPlace180WithCal(wall_presence, speed_rotation);//speed_rotation);

    // go back and go out for maximize correct alignment
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(HALF_CELL_LENGTH);
    basicMove(0, -1.00 * (HALF_CELL_LENGTH - Z3_CENTER_BACK_DIST - UTURN_OFFSET + 15), max_speed, 0);
    while (hasMoveEnded() != TRUE);

    return MAZE_MOVES_E_SUCCESS;
}

/**************************************************************************************/
/***************                    Tests Moves                    ********************/
/**************************************************************************************/
void movesTest1()
{
    int Vout, Vmax, Vrotate;

    HAL_Delay(2000);

#if 0
    Vout = 600;
    Vmax = 600;
    Vrotate = 600;
    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);

    test Uturn
    mazeMoveStartCell(Vmax, Vmax);
    mazeMoveUTurn(Vrotate, Vmax, Vmax);
#endif

#if 1
    Vout = 600;
    Vmax = 600;
    Vrotate = 600;
//    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    //maze
    mazeMoveStartCell(Vmax, Vmax);
    mazeMoveCell(1, Vmax, Vout);
    mazeMoveRotateCCW90(Vrotate, Vout);
    mazeMoveRotateCCW90(Vrotate, Vout);
    mazeMoveUTurn(Vrotate, Vmax, Vout);
    mazeMoveRotateCW90(Vrotate, Vout);
    mazeMoveRotateCW90(Vrotate, Vout);
    mazeMoveCell(1, Vmax, Vout);
    mazeMoveUTurn(Vrotate, Vmax, Vout);
    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveStartCell(Vmax, Vmax);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveUTurn(Vrotate, Vmax, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveUTurn(Vrotate, Vmax, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveUTurn(Vrotate, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveUTurn(Vrotate, Vmax, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(2, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(2, Vmax, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveUTurn(Vrotate, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveRotateCCW90(Vrotate, Vout);
    //    mazeMoveRotateCW90(Vrotate, Vout);
    //    mazeMoveCell(1, Vmax, Vout);
#endif

#if 0
    Vout = 600;
    Vmax = 600;
    Vrotate = 600;
    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    // zigzag
    mazeMoveStartCell(Vmax, Vmax);
    mazeMoveCell(1, Vmax, Vout);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveCell(1, Vmax, Vout);
#endif

    telemetersStop();
}

void movesTest2()
{
    int Vin, Vout, Vmax, Vrotate;

    HAL_Delay(2000);

#if 1
    //rotation test (used for verify move computes)
    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);
    double abs_encoders = encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R);
    Vin  = 10;
    Vout = 0;
    Vmax = 300;

    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);

    mazeMoveRotateInPlaceCW180(200);
    HAL_Delay(2000);
    mazeMoveRotateInPlaceCW180(200);

    //   basicMove(180, 0, Vin, Vin);
    while (hasMoveEnded() != TRUE);
    motorsBrake();
    abs_encoders = (encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R)) - abs_encoders;
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintIntAtLine(0, 2, "reel gyro =  ", (int)gyroGetAngle(), &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 1, "abs  dist =  ", (int)abs_encoders / 2, &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", CELL_LENGTH, &Font_5x8);
    ssd1306Refresh();
    HAL_Delay(2000);
#endif

#if 0
    //distance test (used for verify move computes)
    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);
    double abs_encoders = encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R);
    Vin  = 500;
    Vout = 500;
    Vmax = 1000;
    //    mazeMove(0, 720, Vin, Vin);
    mazeMoveCell(4, Vmax, Vout);
    while (hasMoveEnded() != TRUE);
    motorsBrake();
    abs_encoders = (encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R)) - abs_encoders;
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintIntAtLine(0, 1, "abs  dist =  ", (int)abs_encoders / 2, &Font_5x8);
    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", CELL_LENGTH, &Font_5x8);
    ssd1306Refresh();
    HAL_Delay(2000);
#endif

#if 0
    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);

    double abs_encoders = encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R);
    //test absolute vs relative distance
    mazeMoveStartCell(Vmax, Vmax);
    mazeMoveCell(1, Vmax, Vout);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveCell(2, Vmax, Vout);
    mazeMoveRotateCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    mazeMoveRotateCCW90(Vrotate, Vrotate);
    while (hasMoveEnded() != TRUE);
    abs_encoders = (encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R)) - abs_encoders;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintIntAtLine(0, 1, "abs  dist =  ", (int)abs_encoders / 2, &Font_5x8);
    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", 239 + 179 + 129 + 179 + 179 + 129 + 129 + 129, &Font_5x8);
    ssd1306Refresh();
#endif

    telemetersStop();
    motorsDriverSleep(ON);
}

