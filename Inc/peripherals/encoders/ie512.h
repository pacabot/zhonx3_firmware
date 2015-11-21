/**************************************************************************/
/*!
    @file     IE512.h
    @author   PLF Pacabot.com
    @date     03 August 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __IE512_H__
#define __IE512_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define IE512_DRIVER_E_SUCCESS  0
#define IE512_DRIVER_E_ERROR    MAKE_ERROR(IE512_DRIVER_MODULE_ID, 1)

enum encoderName {ENCODER_L, ENCODER_R};

void 	encodersInit(void);
void 	encoderLeft_IT(void);
void 	encoderRight_IT(void);
int 	encodersReset(void);
double 	encoderGetDist(enum encoderName encoder_name);
void 	encoderTest(void);

#endif //__IE512_H__
