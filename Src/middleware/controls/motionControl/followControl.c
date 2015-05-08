/**************************************************************************/
/*!
    @file    followControl.c
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
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/wall_sensors/wall_sensors.h"

/* Application declarations */
#include "application/lineFollower/lineFollower.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/followControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
follow_control_struct follow_control;
follow_params_struct follow_params;
arm_pid_instance_f32 telemeters_pid_instance;

int followControlInit(void)
{
	telemeters_pid_instance.Kp = 15;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 50;

	follow_control.follow_pid.instance = &telemeters_pid_instance;

	pidControllerInit(follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int followControlLoop(void)
{
	telemetersDistancesTypeDef distances;

	getTelemetersDistance(&distances);
	if ((distances.distance_front_left < 100.00 || distances.distance_front_right < 100.00) &&
			(distances.distance_front_left > 10.00 || distances.distance_front_right > 10.00))
		follow_control.follow_error = distances.distance_front_left - distances.distance_front_right;
	else
		follow_control.follow_error = 0;

	follow_control.follow_command = (pidController(follow_control.follow_pid.instance, follow_control.follow_error));// * (double)follow_params.sign;

	return SPEED_CONTROL_E_SUCCESS;
}

