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
#include "peripherals/tone/tone.h"

/* Middleware declarations */
#include "middleware/display/banner.h"
#include "middleware/safety_stop/safety_stop.h"

/* Declarations for this module */
#include "middleware/powerManagement/powerManagement.h"

#define KILL_DELAY 10000

void batteryGauge_IT(void)
{
    int bat_voltage = 0;

    bat_voltage = multimeterGetBatVoltage();

    if (HAL_GPIO_ReadPin(GPIO_BASE_PORT, GPIO_BASE_PIN) == FALSE)
    {
        bannerSetIcon(USB, TRUE);
    }
    else
    {
        bannerSetIcon(BATTERY, bat_voltage);
    }
    killOnLowBattery(bat_voltage);
}

void killOnLowBattery(int bat_voltage)
{
    static int time = 0;

    if ((bat_voltage < BATTERY_LOWER_VOLTAGE_NO_LOAD) && bat_voltage != 0)
    {
        time++;
        toneItMode(A3, (MULTIMMETER_TIME_FREQ * 1000) / 2);
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(10, 1, &Font_8x8, "LOW BATTERY");
        ssd1306PrintfAtLine(15, 3, &Font_7x8, "V = %d mV", bat_voltage);
        ssd1306Refresh();
        if (((int)(time * (MULTIMMETER_TIME_FREQ )) * 1000) >= KILL_DELAY)
        {
            emergencyStop();
            halt();
        }
    }
    else
        time = 0;
}
