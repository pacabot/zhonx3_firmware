/**************************************************************************/
/*!
 @file    mazeMoves.c
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include <stm32f4xx_hal.h>

/* General declarations */
#include <string.h>
#include <config/basetypes.h>

/* Middleware declarations */
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/positionControl.h>
#include <middleware/controls/mazeControl/wallFollowControl.h>
#include <middleware/moves/basicMoves/basicMoves.h>
#include <middleware/moves/mazeMoves/spyPost.h>
#include <middleware/moves/mazeMoves/spyWall.h>
#include <middleware/wall_sensors/wall_sensors.h>

/* Peripheral declarations */
#include <peripherals/display/smallfonts.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/encoders/ie512.h>
#include <peripherals/gyroscope/adxrs620.h>
#include <peripherals/motors/motors.h>
#include <peripherals/telemeters/telemeters.h>
#include <peripherals/tone/tone.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/bluetooth/bluetooth.h>

/* Declarations for this module */
#include <middleware/moves/mazeMoves/mazeMoves.h>

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
static int mazeMoveFrontAlignment(double max_speed);

/**************************************************************************************/
/***************                    Basic Moves                    ********************/
/**************************************************************************************/

/*
 *
 *      o         o
 *      :         :
 *      :    |    :
 *      o         o
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
 *      o         o
 *      :    |    :
 *      :         :
 *      o         :
 */
int mazeMoveHalfCell_OUT(double max_speed, double end_speed)
{
    while (hasMoveEnded() != TRUE);
    wallFollowSetInitialPosition(HALF_CELL_LENGTH);
    basicMove(0, (HALF_CELL_LENGTH - OFFSET_DIST), max_speed, end_speed); //todo remove groze mede

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

    if (offset->spyPost.right_x != 0)
    {
        offset_error = (double)offset->spyPost.right_x;
#ifdef DEBUG_MAZE_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(", R_OFFSET = %d, TYPE = %d", (int32_t)offset->spyPost.right_x, offset->spyPost.right_spyPostType);
#endif
        toneItMode(C4, 300);
    }
    else if (offset->spyPost.left_x != 0)
    {
        offset_error = (double)offset->spyPost.left_x;
#ifdef DEBUG_MAZE_MOVES
        bluetoothWaitReady();
        bluetoothPrintf(", L_OFFSET = %d, TYPE = %d", (int32_t)offset->spyPost.left_x, offset->spyPost.left_spyPostType);
#endif
        toneItMode(C4, 300);
    }
    else if (offset->frontCal.front_dist != 0)
    {
        offset_error = (double)offset->frontCal.front_dist;
#ifdef DEBUG_MAZE_MOVES
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
    if (offset_error >= (OFFSET_DIST * 3.00))
    {
        while (hasMoveEnded() != TRUE);
        wallFollowSetInitialPosition(CELL_LENGTH - OFFSET_DIST);    //absolute position into a cell
        basicMove(0, (OFFSET_DIST * 2.00), max_speed, max_speed);
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
int mazeMoveFrontAlignment(double max_speed)
{
    double relative_dist = 0.00;
    while (hasMoveEnded() != TRUE);
    if (getWallPresence(FRONT_WALL) == TRUE)
    {
        if (((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) < 21.00)
        {
            basicMove(0, -50.00, 100.00, 100.00);
            while (((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) < 23.00)//todo add in flash
            {
                expanderSetLeds(0b010);
            }
        }
        else
        {
            basicMove(0, 50.00, 100.00, 100.00);
            while (((getTelemeterDist(TELEMETER_FL) + getTelemeterDist(TELEMETER_FR)) / 2.00) > 23.00)//todo add in flash
            {
                expanderSetLeds(0b010);
            }
        }
        basicMove(1, 1, 100.00, 100.00);
        while (hasMoveEnded() != TRUE);
        expanderSetLeds(0b000);
        if (getTelemeterDist(TELEMETER_FR) > getTelemeterDist(TELEMETER_FL))
        {
            toneItMode(A5, 300);
            basicMove(-40.00, 0, 200, 200);
            while (((getTelemeterDist(TELEMETER_FR) - getTelemeterDist(TELEMETER_FL))) > 0.00)
            {
                expanderSetLeds(0b100);
                //                if (hasMoveEnded() == TRUE)
                //                {
                //                    basicMove(40, 0, max_speed, max_speed);
                //                    return MAZE_MOVES_E_SUCCESS;
                //                }
            }
        }
        else
        {
            toneItMode(A5, 300);
            basicMove(40.00, 0, 200, 200);
            while (((getTelemeterDist(TELEMETER_FL) - getTelemeterDist(TELEMETER_FR))) > 0.00)
            {
                expanderSetLeds(0b001);
                //                if (hasMoveEnded() == TRUE)
                //                {
                //                    basicMove(-40, 0, max_speed, max_speed);
                //                    return MAZE_MOVES_E_SUCCESS;
                //                }
            }
        }
        expanderSetLeds(0b000);
        basicMove(0, 0, 0, 0);
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
    double decel_dist = 0;
    double vmax_LastMainDist = 0;
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));

    if (nb_cell < 1)
        return MAZE_MOVES_E_ERROR;

#ifdef DEBUG_MAZE_MOVES
    bluetoothWaitReady();
    bluetoothPrintf("\rMOVE %d CELL", nb_cell);
#endif

    if (nb_cell == 1)
    {
        mazeMoveMainDist(&offset, max_speed, out_speed);
    }
    else
    {
        vmax_LastMainDist = sqrt(2.00 * MAX_ACCEL * MAIN_DIST + pow(out_speed, 2));
        if (max_speed > vmax_LastMainDist)
        {
            while (hasMoveEnded() != TRUE);
            wallFollowSetInitialPosition(OFFSET_DIST); //absolute position into a cell
            basicMove(0, ((double)(nb_cell - 1) * MAIN_DIST) + ((double)(nb_cell - 1) * 2.00 * OFFSET_DIST), max_speed, vmax_LastMainDist);

            mazeMoveMainDist(&offset, vmax_LastMainDist, out_speed);
        }
        else
        {
            while (hasMoveEnded() != TRUE);
            wallFollowSetInitialPosition(OFFSET_DIST); //absolute position into a cell
            basicMove(0, ((double)(nb_cell - 1) * MAIN_DIST) + ((double)(nb_cell - 1) * 2.00 * OFFSET_DIST), max_speed, max_speed);

            mazeMoveMainDist(&offset, max_speed, out_speed);
        }
    }
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

#ifdef DEBUG_MAZE_MOVES
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

#ifdef DEBUG_MAZE_MOVES
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

#ifdef DEBUG_MAZE_MOVES
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

/*       _________
 *      o    _    o
 *      :   / )   :
 *      :   \'    :
 *      o    |    o
 *           v
 */
int mazeMoveUTurn(double speed_rotation, double max_speed, double out_speed)
{
    getOffsetsStruct offset;
    memset(&offset, 0, sizeof(getOffsetsStruct));
    wallSelectorEnum wall_presence = FALSE;

#ifdef DEBUG_MAZE_MOVES
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
    mazeMoveHalfCell_IN(out_speed, 0);
    mazeMoveRotateInPlace180WithCal(wall_presence, speed_rotation);//speed_rotation);
    mazeMoveHalfCell_OUT(out_speed, out_speed);
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

#ifdef DEBUG_MAZE_MOVES
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
    int Vmin, Vmax, Vrotate;

    HAL_Delay(2000);

#if 0
    Vmin = 0;
    Vmax = 400;
    Vrotate = 100;
    //telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);

    basicMove(0, (double)(500), Vmax, Vmin);


    //    mazeMoveStartCell(Vmax, Vmax);
    //    //mazeMoveCell(2, Vmax, Vmin);
    //    mazeMoveCell(1, Vmax, Vmin);
    //    mazeMoveCell(1, Vmax, Vmin);
    //    mazeMoveRotateCW90(Vmin, Vmin);
    //    mazeMoveCell(1, Vmax, 0);

    while( hasMoveEnded() != TRUE);
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", (int)((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00), &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 1, "abs  dist =  ", (int)abs_encoders / 2, &Font_5x8);
    //    ssd1306PrintIntAtLine(0, 2, "reel dist =  ", CELL_LENGTH, &Font_5x8);
    ssd1306Refresh();
    basicMove(0,0,0,0);
    HAL_Delay(3000);

#endif

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
    Vmax = 200;
    Vrotate = 200;
    telemetersStart();

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    //maze
    while(1)
    {
        mazeMoveCell(1, Vmax, Vmax);
        mazeMoveUTurn(Vrotate, Vmax, Vmax);
        mazeMoveCell(1, Vmax, Vmax);
        mazeMoveUTurn(Vrotate, Vmax, Vmax);
    }

    //    mazeMoveStartCell(Vmax, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveUTurn(Vrotate, Vmax, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveUTurn(Vrotate, Vmax, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveRotateCCW90(Vrotate, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveRotateCW90(Vrotate, Vmax);
    //    mazeMoveCell(1, Vmax, Vmax);


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
    motorsDriverSleep(ON);
}

void movesTest2()
{
    int Vin, Vout, Vmax, Vrotate;

    HAL_Delay(2000);

#if 1
    //rotation test (used for verify move computes)
    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(WALL_FOLLOW);
    telemetersStart();

    double abs_encoders = encoderGetAbsDist(ENCODER_L) + encoderGetAbsDist(ENCODER_R);
    Vin  = 10;
    Vout = 0;
    Vmax = 300;

    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);
    //    mazeMoveRotateInPlaceCW90(200);

    //    mazeMoveRotateInPlaceCW180(200);
    //    HAL_Delay(2000);
    //    mazeMoveRotateInPlaceCW180(200);
    //    HAL_Delay(2000);

    mazeMoveUTurn(200, 500, 500);
    HAL_Delay(2000);

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

