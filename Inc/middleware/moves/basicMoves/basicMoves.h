/**************************************************************************/
/*!
 @file    basicMoves.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __BASICMOVES_H__
#define __BASICMOVES_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define BASIC_MOVES_E_SUCCESS  0
#define BASIC_MOVES_E_ERROR    MAKE_ERROR(BASIC_MOVES_MODULE_ID, 1)

//#define DEBUG_BASIC_MOVES

int basicMoveStop(void);
/**
 * @brief compute and start a new movement
 *
 * This function outputs all params for pids controller (speedControl and positionControl).
 *
 * @param
 *
 * angle => angle of rotation in degres (0° for strait move)
 * radius_or_distance => radius in mm of rotate if angle > 0 or distance in mm if angle = 0°
 * max_speed => max speed
 * end_speed => end speed for chain
 *
 * @retval status
 */
int basicMove(double angle, double radius_or_distance, double max_speed, double end_speed);
/**
 * @brief compute and start a new straight movement
 *
 * This function outputs all params for pids controller (speedControl and positionControl).
 *
 * @param
 *
 * angle => angle of rotation in degres (0° for strait move)
 * distance => distance in mm
 * max_speed => max speed
 * end_speed => end speed for chain
 * accel     => acceleration mm/s²
 * @retval status
 */
int basicMoveStraight(double distance, double max_speed, double end_speed, double accel);

#endif // __BASICMOVES_H
