/**************************************************************************/
/*!
    @file    positionControl.c
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

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/positionControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
position_control_struct position_control;

int positionControl_Init(void)
{
//	encoder_pid_instance.Kp = 700.0;
//	encoder_pid_instance.Ki = 0;//0.000001;//0.1;
//	encoder_pid_instance.Kd = 800;//0.4;
//
//	speed_control.current_distance = 0;
//	speed_control.gap_distance_per_loop = 0;
//	speed_control.old_distance = 0;
//	speed_control.current_speed = 0;
//
//	speed_control.speed_error = 0;
//	speed_control.correction = 0;
//	speed_control.speed_consigne = 0;
//
//	speed_control.distance_consigne = 0;
//
//	speed_control.speed.pid_instance = &encoder_pid_instance;
//
//	pidControllerInit(speed_control.speed.pid_instance);
//
//
//	encoderResetDistance(&left_encoder);
//	encoderResetDistance(&right_encoder);
//
	return POSITION_CONTROL_E_SUCCESS;
}


int positionControl(void)
{
	return POSITION_CONTROL_E_SUCCESS;
}
