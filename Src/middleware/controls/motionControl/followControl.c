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
	telemeters_pid_instance.Kp = 3;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 20;

	follow_control.follow_pid.instance = &telemeters_pid_instance;

	pidControllerInit(follow_control.follow_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int followControlLoop(void)
{

//	if (((telemeters.left_diag.telemeter_value > 1000) && (telemeters.right_diag.telemeter_value > 1000)) && (follow_params.active_state == 1))
//	if (((telemeters.left_front.telemeter_values > 1000) && (telemeters.right_front.telemeter_values > 1000)) && (follow_params.active_state == 1))

//	{
		follow_control.follow_error = (telemeters.right_front.value_average - telemeters.left_front.value_average);

//	{
//		follow_control.follow_error = (telemeters.left_diag.value_average - telemeters.right_diag.value_average);

		follow_params.sign = SIGN(follow_control.follow_error);
		follow_control.follow_error = fabsf(follow_control.follow_error);

		follow_control.follow_command = (pidController(follow_control.follow_pid.instance, follow_control.follow_error)) * (float)follow_params.sign;
//	}
//	else
//	{
//		follow_control.follow_command = 0;
//	}
	return SPEED_CONTROL_E_SUCCESS;
}

