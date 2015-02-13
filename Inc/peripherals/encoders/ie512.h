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

// Machine Definitions
typedef struct
{
    signed int nb_revolutions;              // relative mouvement counter
//    int nb_increments;
    signed char *direction;
} ENCODER_DEF;

void encoders_Init(void);
void encoderLeft_IT(void);
void encoderRight_IT(void);
void encoderTest(void);

#endif
