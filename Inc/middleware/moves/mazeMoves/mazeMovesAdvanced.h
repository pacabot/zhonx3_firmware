/**************************************************************************/
/*!
 @file    mazeMoveAdvanced.h
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
#ifndef __MAZEMOVESADVANCED_H__
#define __MAZEMOVESADVANCED_H__

/* Module Identifier */
#include <config/module_id.h>
#include <config/errors.h>

/* Error codes */
#define MAZE_MOVES_ADVANCED_E_SUCCESS  0
#define MAZE_MOVES_ADVANCED_E_ERROR    MAKE_ERROR(MAZE_MOVES_ADVANCED_MODULE_ID, 1)

int mazeMoveAdvancedTest(void);

#endif // __MAZEMOVESADVANCED_H
