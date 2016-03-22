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

int  	rotate180WithCal(enum rotationTypeEnum rotation_type, float max_speed, float end_speed);
int  	rotate90WithCal(enum rotationTypeEnum rotation_type, float max_speed, float end_speed);
int  	moveCell(unsigned long nb_cell, float max_speed, float end_speed);
int  	moveHalfCell_IN(float max_speed, float end_speed);
int  	moveHalfCell_OUT(float max_speed, float end_speed);
int  	moveEndCell(float max_speed, float end_speed);
int  	moveStartCell(float max_speed, float end_speed);
int  	moveRotateCW90(float max_speed, float end_speed);
int  	moveRotateCCW90(float max_speed, float end_speed);
int  	moveUTurn(float speed_rotation, float max_speed, float end_speed);
void 	movesTest(void);
void 	rotateTest(void);

#endif // __BASICMOVES_H
