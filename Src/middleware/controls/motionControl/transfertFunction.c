/**************************************************************************/
/*!
    @file    transfertFunction.c
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
#include "peripherals/motors/motors.h"
#include "peripherals/multimeter/multimeter.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/followControl.h"

/* Declarations for this module */
#include "middleware/controls/motionControl/transfertFunction.h"

transfert_function_struct transfert_function;

int transfertFunctionInit(void)
{
	transfert_function.left_motor_pwm = 0;
	transfert_function.right_motor_pwm = 0;
	transfert_function.pwm_ratio = 0;
	return TRANSFERT_FUNCTION_E_SUCCESS;
}

int transfertFunctionLimiter(void)
{
	if (transfert_function.left_motor_pwm > MOTORS_PERIOD)
		transfert_function.left_motor_pwm = MOTORS_PERIOD;

	if (transfert_function.left_motor_pwm < -MOTORS_PERIOD)
		transfert_function.left_motor_pwm = -MOTORS_PERIOD;

	if (transfert_function.right_motor_pwm > MOTORS_PERIOD)
		transfert_function.right_motor_pwm = MOTORS_PERIOD;

	if (transfert_function.right_motor_pwm < -MOTORS_PERIOD)
		transfert_function.right_motor_pwm = -MOTORS_PERIOD;

	return TRANSFERT_FUNCTION_E_SUCCESS;
}

int transfertFunctionLoop(void)
{
	if (multimeter.vbat.value > 6000 && multimeter.vbat.value < 9000)
		transfert_function.pwm_ratio = (PWM_RATIO_COEFF_A * multimeter.vbat.value + PWM_RATIO_COEFF_B);	//compute ratio to not exceed motor voltage
	else
		transfert_function.pwm_ratio = (PWM_RATIO_COEFF_A * 8000 + PWM_RATIO_COEFF_B);						//if vbat read fail

	transfert_function.right_motor_pwm = (speed_control.speed_command - (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;
	transfert_function.left_motor_pwm  = (speed_control.speed_command + (position_control.position_command + follow_control.follow_command)) * transfert_function.pwm_ratio;

	transfertFunctionLimiter();
	motorSet(&right_motor, transfert_function.right_motor_pwm, DECAY_FAST);
	motorSet(&left_motor, transfert_function.left_motor_pwm, DECAY_FAST);

	return TRANSFERT_FUNCTION_E_SUCCESS;
}
