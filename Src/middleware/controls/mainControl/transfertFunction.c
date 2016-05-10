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

/* Middleware declarations */
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/times_base/times_base.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"

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
    {
        transfert_function.left_motor_pwm = MOTORS_PERIOD;
        transfert_function.right_motor_pwm -= (transfert_function.left_motor_pwm - MOTORS_PERIOD);
    }

    if (transfert_function.left_motor_pwm < -MOTORS_PERIOD)
    {
        transfert_function.left_motor_pwm = -MOTORS_PERIOD;
        transfert_function.right_motor_pwm -= (transfert_function.left_motor_pwm + MOTORS_PERIOD);
    }

    if (transfert_function.right_motor_pwm > MOTORS_PERIOD)
    {
        transfert_function.right_motor_pwm = MOTORS_PERIOD;
        transfert_function.left_motor_pwm -= (transfert_function.right_motor_pwm - MOTORS_PERIOD);
    }

    if (transfert_function.right_motor_pwm < -MOTORS_PERIOD)
    {
        transfert_function.right_motor_pwm = -MOTORS_PERIOD;
        transfert_function.left_motor_pwm -= (transfert_function.right_motor_pwm + MOTORS_PERIOD);
    }

    return TRANSFERT_FUNCTION_E_SUCCESS;
}

int transfertFunctionLoop(void)
{
    //	if (multimeterGetBatVoltage() > 5000 && multimeterGetBatVoltage() < 9000)
    //		transfert_function.pwm_ratio = (PWM_RATIO_COEFF_A * multimeterGetBatVoltage() + PWM_RATIO_COEFF_B);	//compute ratio to not exceed motor voltage
    //	else
    transfert_function.pwm_ratio = (PWM_RATIO_COEFF_A * 8000.00 + PWM_RATIO_COEFF_B);	//if vbat read fail

    transfert_function.right_motor_pwm = (speedControlGetSpeedCommand()
            - (positionControlGetPositionCommand() + wallFollowGetCommand() + line_follow_control.line_follow_command))
                    * transfert_function.pwm_ratio;
    transfert_function.left_motor_pwm = (speedControlGetSpeedCommand()
            + (positionControlGetPositionCommand() + wallFollowGetCommand() + line_follow_control.line_follow_command))
                    * transfert_function.pwm_ratio;

    transfertFunctionLimiter();
    motorSet_DF(MOTOR_R, transfert_function.right_motor_pwm);
    motorSet_DF(MOTOR_L, transfert_function.left_motor_pwm);
    if (motorGetFault() != MOTORS_DRIVER_E_SUCCESS)
        ledPowerErrorBlink(20, 20, 20);
    else
        ledPowerErrorBlink(0, 0, 0);
    //	bluetoothPrintf("pwmR: %d \r\n", (transfert_function.right_motor_pwm));
    return TRANSFERT_FUNCTION_E_SUCCESS;
}
