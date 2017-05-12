/*
 * safety_stop.c
 *
 *  Created on: 10 mai 2017
 *      Author: zhonx
 */

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
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/motors/motors.h"
#include "peripherals/times_base/times_base.h"

/* Middleware declarations */

/* Declarations for this module */
#include "middleware/safety_stop/safety_stop.h"


void emergencyStop()
{
    telemetersStop();
    motorsDriverSleep(ON);
    motorsBrake();
}

// Shutdown the robot
void halt(void)
{
    GPIO_InitTypeDef gpio;

    gpio.Pin = GPIO_PW_KILL_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIO_PW_KILL_PORT, &gpio);

    HAL_GPIO_WritePin(GPIO_PW_KILL_PORT, GPIO_PW_KILL_PIN, GPIO_PIN_RESET);
}
