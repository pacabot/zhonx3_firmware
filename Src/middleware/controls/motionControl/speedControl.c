/**************************************************************************/
/*!
    @file    speedControl.c
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

/* Declarations for this module */
#include "middleware/controls/motionControl/speedControl.h"

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */
speed_control_struct speed_control;

int speedControl_Init(void)
{
	int rv;
	speed_control.acc_cnt = 0;
	speed_control.dcc_cnt = 0;
	speed_control.maintain_cnt = 0;
	speed_control.old_speed2 = 0;
	speed_control.current_speed2 = 0;
	speed_control.maintain_speed = 0;
	speed_control.step_distance = 0;
	speed_control.mm_distance = 0;

	return SPEED_CONTROL_E_SUCCESS;
}

int speedControl(void)
{
	int rv;
	int current_cnt;

	current_cnt = speed_control.old_cnt - (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder) / 2);


	return SPEED_CONTROL_E_SUCCESS;
}

int speedAcc(uint32_t initial_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedDcc(uint32_t final_speed, uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}

int speedMaintain(uint32_t distance)
{
	int rv;
	return SPEED_CONTROL_E_SUCCESS;
}
