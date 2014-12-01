/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private variables ---------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim10;

//#include "TimesBase.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/encoders/ie512.h"
#include "controls/pid/pid.h"
#include "times_base/times_base.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if( htim == &htim10)
	  {
		  LineSensors_IT();
//		  Telemeters_IT();
	  }
	  if( htim == &htim7)
	  {
		  High_Freq_IT();
	  }
	  if( htim == &htim6)
	  {
		  Low_Freq_IT();
	  }
	  if( htim == &htim1)
	  {
		  Left_Encoder_IT();
	  }
	  if( htim == &htim3)
	  {
		  Right_Encoder_IT();
	  }
}
