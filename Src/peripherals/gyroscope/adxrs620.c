/**************************************************************************/
/*!
 @file    ADXRS620.c
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/

/* General declarations */
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_adc_ex.h>
#include <stm32f4xx_hal_gpio.h>
#include <stdint.h>

/* Peripheral declarations */
#include <peripherals/display/smallfonts.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/flash/flash.h>
#include <peripherals/motors/motors.h>

/* Middleware declarations */
#include <middleware/display/banner.h>
#include <middleware/moves/basicMoves/basicMoves.h>
#include <middleware/settings/settings.h>
#include <middleware/controls/mainControl/positionControl.h>

/* Declarations for this module */
#include <peripherals/gyroscope/adxrs620.h>

/* Types definitions */
typedef struct
{
    uint16_t adc_value;
    uint32_t callback_cnt;
    volatile double current_angle;
} gyro_struct;

volatile gyro_struct gyro;

extern ADC_HandleTypeDef hadc1;

__IO uint32_t DMA_ADC_Gyro_Rate;

volatile double gyro_Current_Angle = 0.0;

GPIO_InitTypeDef GPIO_InitStruct;
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void adxrs620Init(void)
{
    ADC_InjectionConfTypeDef sConfigInjected;
    bannerSetIcon(GYROMETER, TRUE);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
     */
    sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
    sConfigInjected.InjectedRank = 1;
    sConfigInjected.InjectedNbrOfConversion = 1;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_15CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
    sConfigInjected.AutoInjectedConv = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;
    HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
}

/*## ADC Gyroscope callback for angle computing  #################################*/
/* -----------------------------------------------------------------------
 Use TIM5 for start Injected conversion on ADC1 (gyro rate).
 ----------------------------------------------------------------------- */
void adxrs620_ADC_IT(void)
{
    gyro.current_angle += (GYRO_A_COEFF * (gyro.adc_value = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1)))
            - zhonxCalib_data->gyro.calib_value;  //optimized gyro integration DMA

    gyro.callback_cnt++;
}

double adxrs620Calibrate(int nb_ech)
{
    double sample_sum = 0;
    for (int i = 0; i < nb_ech; i++)
    {
        sample_sum += GYRO_A_COEFF * gyro.adc_value;
        HAL_Delay(1);
    }
    return sample_sum / (double) nb_ech;
}

double gyroGetAngle(void)
{
    return gyro.current_angle;
}

void gyroResetAngle(void)
{
    gyro.current_angle = 0;
//	adxrs620Calibrate(10);
}

void adxrs620Cal(void)
{
    double cal;
    int rv;
    adxrs620Init();

    positionControlSetPositionType(ENCODERS);
    motorsDriverSleep(OFF);
    basicMove(0,0,0,0);

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(0, 1, "DON'T TOUTCH Z3!!", &Font_3x6);
    ssd1306Refresh();
    HAL_Delay(3000);
    cal = adxrs620Calibrate(5000);
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&(zhonxCalib_data->gyro.calib_value),
                     (unsigned char *)&cal, sizeof(double));
    if (rv != FLASH_DRIVER_E_SUCCESS)
    {
        ssd1306PrintfAtLine(0, 2, &Font_5x8, "FAILED To write calibration value");
        ssd1306Refresh();
        HAL_Delay(2000);
    }
    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintIntAtLine(10, 3, "B =", (int32_t) (cal * 100000.00), &Font_5x8);
        ssd1306Refresh();
    }
    motorsDriverSleep(ON);
}

void adxrs620Test(void)
{
    adxrs620Init();
    gyroResetAngle();

    while (expanderJoyFiltered() != JOY_LEFT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintIntAtLine(0, 0, "Angle =  ", (int32_t) gyroGetAngle(), &Font_5x8);
        ssd1306PrintIntAtLine(0, 1, "cnt =  ", (int32_t) gyro.callback_cnt / 1000, &Font_5x8);

        ssd1306PrintIntAtLine(0, 3, "ADC val =  ", (int32_t) gyro.adc_value, &Font_5x8);

        ssd1306Refresh();
    }
}
