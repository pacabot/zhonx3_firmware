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
#include "peripherals/telemeters/telemeters.h"

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
arm_pid_instance_f32 telemeters_pid_instance;

int positionControlInit(void)
{
	telemeters_pid_instance.Kp = 1;
	telemeters_pid_instance.Ki = 0;
	telemeters_pid_instance.Kd = 50;

	position_control.current_angle = 0;
	position_control.gap_orientation_per_loop = 0;
	position_control.old_orientation = 0;
	position_control.current_angular_speed = 0;
	position_control.rotation_consigne = 0;
	position_control.rotation_error = 0;
	position_control.position_command = 0;
	position_control.position_pid.instance = &telemeters_pid_instance;

	pidControllerInit(position_control.position_pid.instance);

	return POSITION_CONTROL_E_SUCCESS;
}

int positionControlLoop(void)
{
//	int rv;

	position_control.rotation_error = (telemeters.left_diag.telemeter_value - telemeters.right_diag.telemeter_value);
//	if ((telemeters.left_diag.telemeter_value - telemeters.right_diag.telemeter_value) > 100);
		position_control.position_command = pidController(position_control.position_pid.instance, position_control.rotation_error);
//		if (position_control.position_command > 200)
//		position_control.position_command = 200;

	return POSITION_CONTROL_E_SUCCESS;
}
