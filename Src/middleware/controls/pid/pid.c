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

#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"

#include <arm_math.h>

#include "middleware/controls/straight_controls/straight_controls.h"
#include "middleware/controls/pid/pid.h"

/* extern variables ---------------------------------------------------------*/
/* global variables ---------------------------------------------------------*/

GPIO_InitTypeDef GPIO_InitStruct;

void pidInit(arm_pid_instance_f32 * instance);
void pidReset(arm_pid_instance_f32 * instance);
float32_t pid(arm_pid_instance_f32 * instance, float32_t error);

void pidInit(arm_pid_instance_f32 * instance)
{
	arm_pid_init_f32(instance, 1);
}

void pidReset(arm_pid_instance_f32 * instance)
{
	arm_pid_reset_f32(instance);
}

float32_t pid(arm_pid_instance_f32 * instance, float32_t error)
{
	return (arm_pid_f32(instance, error));
}

void pids_IT(void)
{
	straightControl_IT();
}
