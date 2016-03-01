/**************************************************************************/
/*!
    @file     powerManagment.c
    @author   PLF Pacabot.com
    @date     03 January 2016
    @version  0.10
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
#include "peripherals/multimeter/multimeter.h"

/* Middleware declarations */
#include "middleware/display/banner.h"

/* Declarations for this module */
#include "middleware/powerManagment/powerManagment.h"

void batteryGauge_IT(void)
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_BASE) == FALSE)
	{
		bannerSetIcon(USB, TRUE);
	}
	else
	{
		bannerSetIcon(BATTERY, (int)(multimeterGetBatVoltage() / BATTERY_COEFF_A - BATTERY_COEFF_B));
	}
}
