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

//#include "stdbool.h"
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
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/positionControl.h"

/* Declarations for this module */
#include "middleware/controls/mainControl/mainControl.h"

int mainControlLoop(void)
{
	int rv;

	UNUSED(rv);

	speedControl();
	positionControl();

	return MAIN_CONTROL_E_SUCCESS;
}

void mainControlTest(void)
{

}

