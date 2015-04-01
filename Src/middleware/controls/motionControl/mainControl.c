/**************************************************************************/
/*!
    @file    mainControl.c
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
#include <middleware/controls/motionControl/transfertFunction.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/mainControl.h"

int mainControlInit(void)
{
	motorsInit();
	encodersInit();
	mulimeterInit();
	telemetersInit();
	speedControlInit();
	positionControlInit();
	transfertFunctionInit();

	return MAIN_CONTROL_E_SUCCESS;
}

int mainControlLoop(void)
{
	speedControlLoop();
//	positionControlLoop();
	transfertFunctionLoop();

	return MAIN_CONTROL_E_SUCCESS;
}

void straightMove(float distance, enum speedRate speed_rate)
{
	speedProfileCompute(distance, speed_rate);
	encoderResetDistance(&left_encoder);
	encoderResetDistance(&right_encoder);
	pid_loop.start_state = TRUE;
	motorsSleepDriver(OFF);
}

void mainControlTest(void)
{
	mainControlInit();
	//HAL_Delay(500);
	straightMove(500, MEDIUMSPEED);

	while(expanderJoyState()!=LEFT)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10,  5,  "dist_c =  ",(int) (speed_control.current_distance * 100), &Font_5x8);
		ssd1306PrintInt(10,  15, "acc dist =  ",(int16_t) speed_control.speed_consign, &Font_5x8);
		ssd1306PrintInt(10,  25, "max speed =  ",(int16_t) speed_params.max_speed, &Font_5x8);
		ssd1306PrintInt(10,  35, "error =  ",(int16_t) speed_control.speed_error, &Font_5x8);
		ssd1306PrintInt(10,  45, "left PWM =  ",(int16_t) transfert_function.left_motor_pwm, &Font_5x8);
		ssd1306PrintInt(10,  55, "right PWM =  ",(int16_t) transfert_function.right_motor_pwm, &Font_5x8);

		ssd1306Refresh();
	}
	antiBounceJoystick();
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

