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
#define IE512_DRIVER_MODULE_ID  5

/* Error codes */
#define IE512_DRIVER_E_SUCCESS  0
#define IE512_DRIVER_E_ERROR    MAKE_ERROR(IE512_DRIVER_MODULE_ID, 1)

// Machine Definitions
typedef struct
{
    volatile float abs_dist;
    volatile float offset_dist;
    volatile float rel_dist;
    signed int mot_rev_cnt;
	TIM_HandleTypeDef *timer;
} encoder;

extern encoder left_encoder;
extern encoder right_encoder;

void encodersInit(void);
void encoderLeft_IT(void);
void encoderRight_IT(void);
void encoderTest(void);
int encoderResetDistance(encoder *enc);
float encoderGetDistance(encoder *enc);

#endif
