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
#include "peripherals/gyroscope/adxrs620.h"

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
	motorsInit();
	encodersInit();
	adxrs620Init();
	gyroResetAngle();

	HAL_Delay(1000);

	uint16_t buff[10000] = {0};
	double speed = 0.00;
	const int refresh_frequency = 1000;
	double current_distance = 0.00;
	double current_angle = 0.00;
	double old_distance = 0.00;
	double old_angle = 0.00;

	int i = 0;

	motorsInit();
	encodersInit();

	HAL_Delay(1000);

	// Forward Fast (PWM on IN1, LOW on IN2)
	motorSet_DF(MOTOR_L, 200);
	motorSet_DF(MOTOR_R, -200);
	motorsDriverSleep(OFF);

	while((int)current_angle < 400)//((int)speed < 3000 && (int)current_distance < 4000)
	{
		current_angle = gyroGetAngle();

		speed = ((current_angle - old_angle) * (double)refresh_frequency); //actual speed (mm/s)

		buff[i] = (uint16_t)speed;
		i++;

		HAL_Delay(1000/refresh_frequency);
		old_distance = current_angle;
	}

	motorsBrake();
	motorsDriverSleep(ON);

	bluetoothSend((uint8_t*)buff, i*2);

	return;

	// Forward Fast (PWM on IN1, LOW on IN2)
	motorSet_DF(MOTOR_L, 100);
	motorSet_DF(MOTOR_R, 100);
	motorsDriverSleep(OFF);

	while((int)current_distance < 400)//((int)speed < 3000 && (int)current_distance < 4000)
	{
		current_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;

		speed = ((current_distance - old_distance) * (double)refresh_frequency); //actual speed (mm/s)

		buff[i] = (uint16_t)speed;
		i++;

		HAL_Delay(1000/refresh_frequency);
		old_distance = current_distance;
	}

	motorsBrake();
	motorsDriverSleep(ON);

	bluetoothSend((uint8_t*)buff, i*2);
}

void pidCalculator(void)
{
	accelMotor_GetStepResponse();
}
