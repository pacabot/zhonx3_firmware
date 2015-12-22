/**************************************************************************/
/*!
    @file    pidCalculator.c
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
#include "peripherals/encoders/ie512.h"
#include "peripherals/motors/motors.h"
#include "peripherals/tone/tone.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/speedControl.h"

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */

/* private variables */


void accelMotor_GetStepResponse(void)
{
	double speed;
	const int refresh_frequency = 1000;
	double current_distance;
	double old_distance;

	motorsInit();

	// Forward Fast (PWM on IN1, LOW on IN2)
	motorSet_DF(MOTOR_L, 50);
	motorSet_DF(MOTOR_R, 50);
	motorsDriverSleep(OFF);

	while(1)
	{
		int speedCompute(void)
		{
			current_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;

			speed = ((current_distance - old_distance) * refresh_frequency); //actual speed (mm/s)

			return SPEED_CONTROL_E_SUCCESS;
		}
		bluetoothPrintf("%d", speed);
		HAL_Delay(1*1000/refresh_frequency);
		old_distance = current_distance;
	}
}

void pidCalculator(void)
{
	accelMotor_GetStepResponse();
}
