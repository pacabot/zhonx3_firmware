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

/* Middleware declarations */

/* Declarations for this module */
#include "peripherals/callback/user_it_callback.h"

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
//extern TIM_HandleTypeDef htim10;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

//#include "TimesBase.h"
#include "config/basetypes.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/encoders/ie512.h"
#include <middleware/controls/pidController/pidController.h>
#include "peripherals/times_base/times_base.h"

/* TIM callback --------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if( htim == &htim2)
	  {
		  if (telemeters.active_state == TRUE)
			  telemeters_IT();
	  }
	  if (htim == &htim5)
	  {
		  if (lineSensors.active_state == TRUE)
		  lineSensors_IT();
	  }
	  if (htim == &htim7)
	  {
		  highFreq_IT();
	  }
	  if (htim == &htim6)
	  {
		  lowFreq_IT();
	  }
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
	  if( htim == &htim4)
	  {
		  multimeter_IT();
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
	if (hadc == &hadc1)
	{
		multimeter_ADC_IT();
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc == &hadc2)
	{
		telemeters_ADC2_IT();
	}
	if (hadc == &hadc3)
	{
		telemeters_ADC3_IT();
	}
}
