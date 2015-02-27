/**************************************************************************/
/*!
    @file     PID.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#include "stdbool.h"
#include <arm_math.h>

#include "stm32f4xx_hal.h"

#include "config/basetypes.h"

#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"

#include <arm_math.h>
#include <middleware/controls/pidController/pidController.h>
#include "middleware/controls/straight_controls/straight_controls.h"

/* extern variables ---------------------------------------------------------*/
extern struct control control;
/* global variables ---------------------------------------------------------*/

GPIO_InitTypeDef GPIO_InitStruct;

void pidControllerInit(arm_pid_instance_f32 * instance);
void pidControllerReset(arm_pid_instance_f32 * instance);
float32_t pidController(arm_pid_instance_f32 * instance, float32_t error);

void pidControllerInit(arm_pid_instance_f32 * instance)
{
	arm_pid_init_f32(instance, 1);
}

void pidControllerReset(arm_pid_instance_f32 * instance)
{
	arm_pid_reset_f32(instance);
}

float32_t pidController(arm_pid_instance_f32 * instance, float32_t error)
{
	return (arm_pid_f32(instance, error));
}

void pidController_IT(void)
{
	if (control.start_state == TRUE)
		straightControl_IT();
}
