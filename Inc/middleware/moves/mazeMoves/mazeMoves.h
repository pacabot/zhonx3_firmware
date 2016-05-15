/**************************************************************************/
/*!
 @file    mazeMove.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __MAZEMOVES_H__
#define __MAZEMOVES_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define MAZE_MOVES_E_SUCCESS  0
#define MAZE_MOVES_E_ERROR    MAKE_ERROR(MAZE_MOVES_MODULE_ID, 1)

//#define DEBUG_BASIC_MOVES

typedef struct
{
    double max_speed_rotation;
    double max_speed_translation;
    double end_speed_translation;
}moveSpeedStruct;

int  mazeMoveCell(unsigned int nb_cell, double max_speed, double out_speed);
int  mazeMoveEndCell(double max_speed, double out_speed);
int  mazeMoveStartCell(double max_speed, double out_speed);
int  mazeMoveRotateCW90(double max_speed, double out_speed);
int  mazeMoveRotateCCW90(double max_speed, double out_speed);
int  mazeMoveRotateInPlace180WithCal(wallSelectorEnum wall_presence, double speed_rotation);
int  mazeMoveRotateInPlaceWithCalCW90(double speed_rotation);
int  mazeMoveRotateInPlaceWithCalCCW90(double speed_rotation);
int  mazeMoveUTurn(double speed_rotation, double max_speed, double out_speed);
int  mazeMoveResetStart(double speed_rotation, double max_speed, double out_speed);
void mazeMovesTest1(void);
void mazeMovesTest2(void);

#endif // __MAZEMOVES_H
