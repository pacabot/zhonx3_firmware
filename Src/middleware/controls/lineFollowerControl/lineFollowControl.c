/**************************************************************************/
/*!
 @file    lineFollowControl.c
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

/* Middleware declarations */
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "application/lineFollower/lineFollower.h"

/* Declarations for this module */

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
line_follow_control_struct line_follow_control;
line_follow_params_struct line_follow_params;
arm_pid_instance_f32 line_follow_pid_instance;
int Line_follower_KP = 2000;
int Line_follower_KD = 80000; //810

int lineFollowControlInit(void)
{

    line_follow_pid_instance.Kp = Line_follower_KP;
    line_follow_pid_instance.Ki = 0;
    line_follow_pid_instance.Kd = Line_follower_KD;

    line_follow_control.line_follow_pid.instance = &line_follow_pid_instance;

    line_follow_control.succes = FALSE;

    pidControllerInit(line_follow_control.line_follow_pid.instance);

    return LINE_FOLLOW_CONTROL_E_SUCCESS;
}

int lineFollowControlLoop(void)
{
    line_follow_control.line_follow_error = line_follower.position;

    line_follow_control.line_follow_command = (pidController(line_follow_control.line_follow_pid.instance,
                                                             line_follow_control.line_follow_error));

    return LINE_FOLLOW_CONTROL_E_SUCCESS;
}

