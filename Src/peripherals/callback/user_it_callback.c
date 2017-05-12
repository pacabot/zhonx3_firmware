/* Includes ------------------------------------------------------------------*/
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
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/times_base/times_base.h"
#include "peripherals/tone/tone.h"
#include "peripherals/motors/motors.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"

/* Middleware declarations */
#include "middleware/powerManagement/powerManagement.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/safety_stop/safety_stop.h"
#include <middleware/controls/mainControl/positionControl.h>
#include <middleware/controls/mainControl/speedControl.h>

/* Application declarations */
#include "application/lineFollower/lineFollower.h"

/* Declarations for this module */
#include "peripherals/callback/user_it_callback.h"

/* Extern variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
//extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern IWDG_HandleTypeDef hiwdg;

/* TIM callback --------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#ifdef DEDICATED_TIMER
    if( htim == &htim2)
    {
        telemeters_IT();
    }
    if (htim == &htim7)	//hight time freq
    {
        mainControl_IT();
    }
#else
    static uint32_t cnt = 0;
    static int32_t rv = MAIN_CONTROL_E_SUCCESS;
    if (htim == &htim7) //hight time freq
    {
        cnt++;
        if (cnt % (int)(HI_TIME_FREQ / TELEMETERS_TIME_FREQ) == 0)
        {
            telemeters_IT();
        }
        if (cnt % (int)(HI_TIME_FREQ / CONTROL_TIME_FREQ) == 0)
        {
            if (rv == MAIN_CONTROL_E_SUCCESS)
            {
                rv = mainControl_IT();
                HAL_IWDG_Refresh(&hiwdg);
            }
            else
            {
                emergencyStop();
                HAL_TIM_Base_Stop_IT(&htim7);

                toneItMode(A4, 3000);
                ssd1306ClearScreen(MAIN_AREA);
                ssd1306PrintfAtLine(5, 1, &Font_8x8, "CONTROL ERROR");
                if (rv == POSITION_CONTROL_E_ERROR)
                    ssd1306PrintfAtLine(20, 3, &Font_7x8, "Position");
                else if (rv == POSITION_CONTROL_E_ERROR)
                    ssd1306PrintfAtLine(30, 3, &Font_7x8, "Speed");

                ssd1306PrintfAtLine(15, 4, &Font_7x8, "rv = %d", rv);
                ssd1306Refresh();
            }
        }
        if (cnt % (int)(HI_TIME_FREQ / LINESENSORS_TIME_FREQ) == 0)
        {
            lineSensors_IT();
        }
        if (cnt % (int)(HI_TIME_FREQ / LINE_FOLLOWER_TIME_FREQ) == 0)
        {
            lineFollower_IT();
        }
        //        if (cnt % (int)(HI_TIME_FREQ / LOW_TIME_FREQ) == 0)
        //        {
        //            tone_IT();
        //            //sleep_mode_IT();
        //            ledBlink_IT();
        //        }
    }
    if (htim == &htim6) //low time freq
    {
        tone_IT();
        //sleep_mode_IT();
        ledBlink_IT();
    }
#endif
    if (htim == &htim1)
    {
        encoderLeft_IT();
    }
    if (htim == &htim3)
    {
        encoderRight_IT();
    }
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim4)
    {
        multimeter_IT();
        batteryGauge_IT();
    }
}

/* ADC callback --------------------------------------------------------------*/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc == &hadc1)
    {
        adxrs620_ADC_IT();
    }
    if (hadc == &hadc2 || hadc == &hadc3)
    {
        lineSensors_ADC_IT(hadc);
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    //    if (hadc == &hadc1)
    //    {
    //        multimeter_ADC_IT();
    //    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc == &hadc1)
    {
        multimeter_ADC_IT();
    }
    if (hadc == &hadc2)
    {
        telemeters_ADC2_IT();
    }
    if (hadc == &hadc3)
    {
        telemeters_ADC3_IT();
    }
}
