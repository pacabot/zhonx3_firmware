/**************************************************************************/
/*!
 @file    mainControl.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __BASICMOVES_H__
#define __BASICMOVES_H__

#include <middleware/controls/mainControl/speedControl.h>
#include <middleware/controls/mainControl/mainControl.h>
#include "config/config.h"
#include "middleware/wall_sensors/wall_sensors.h"

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define BASIC_MOVES_E_SUCCESS  0
#define BASIC_MOVES_E_ERROR    MAKE_ERROR(BASIC_MOVES_MODULE_ID, 1)

//#define DEBUG_BASIC_MOVES

int moveCell(unsigned int nb_cell, double max_speed, double end_speed);
int moveEndCell(double max_speed, double end_speed);
int moveStartCell(double max_speed, double end_speed);
int moveRotateCW90(double max_speed, double end_speed);
int moveRotateCCW90(double max_speed, double end_speed);
int rotate180WithCal(wallSelectorEnum wall_presence, double speed_rotation);
int rotateInPlaceWithCalCW90(wallSelectorEnum wall_presence, double speed_rotation);
int rotateInPlaceWithCalCCW90(wallSelectorEnum wall_presence, double speed_rotation);
int moveUTurn(double speed_rotation, double max_speed, double end_speed);
void movesTest(void);
void rotateTest(void);
void mainControlDisplayTest(void);

#endif // __BASICMOVES_H
