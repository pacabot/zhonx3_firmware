/**************************************************************************/
/*!
    @file     tests.c
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "peripherals/tests/tests.h"

#include "peripherals/display/smallfonts.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/pictures.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/lineSensors/lineSensors.h"

#include "math.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern I2C_HandleTypeDef hi2c1;

GPIO_InitTypeDef GPIO_InitStruct;

/* Variable used to get converted value */
__IO uint32_t uhADCxConvertedValue_1 = 0;
__IO uint32_t uhADCxConvertedValue_2 = 0;
__IO uint32_t uhADCxConvertedValue_3 = 0;
__IO uint32_t uhADCxConvertedValue_4 = 0;
__IO uint32_t uhADCxConvertedValue_5 = 0;

__IO uint32_t uhADCxConvertedValue_6 = 0;
__IO uint32_t uhADCxConvertedValue_7 = 0;
__IO uint32_t uhADCxConvertedValue_8 = 0;

extern volatile int  L_diag_distance;
extern volatile int  R_diag_distance;
extern volatile int  L_front_distance;
extern volatile int  R_front_distance;

extern volatile float gyro_Current_Angle;
extern __IO uint32_t uhADCxConvertedValue_Gyro;

/**************************************************************************/
/* Structure init                                                 */
/**************************************************************************/
// Global variable
extern ENCODER_DEF left_encoder;   // left encoder structure
extern ENCODER_DEF right_encoder;   // right encoder structure

void test_EasterEgg(void)
{

	// I2C
	uint8_t aTxBuffer[3]; // = {control, c};
	aTxBuffer[0] = 0x0;
	aTxBuffer[1] = 0x1;
	aTxBuffer[2] = 0x5;

	uint8_t aRxBuffer[1]; // = {control, c};
	aRxBuffer[0] = 0x0;

	ssd1306ClearScreen();

	while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0x50<<1, (uint8_t*)aTxBuffer, 3, 10000)!= HAL_OK)
	{
	  /* Error_Handler() function is called when Timout error occurs.
		 When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
		 Master restarts communication */
	  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	  {
		  ssd1306PrintInt(10,  15, "Bad Send", 0, &Font_5x8);
	  }

	}

	while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)0x50<<1, (uint8_t*)aRxBuffer, 1, 10000)!= HAL_OK)
	{
	  /* Error_Handler() function is called when Timout error occurs.
		 When Acknowledge failure ocucurs (Slave don't acknowledge it's address)
		 Master restarts communication */
	  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
	  {
		  ssd1306PrintInt(10,  25, "Bad recv", 0, &Font_5x8);
	  }

	}

	ssd1306PrintInt(10,  35, "Fini : ", aRxBuffer[0], &Font_5x8);
	ssd1306Refresh();

}

float currentAngle = 0.0;          //Keep track of our current angle
float gyroRate;

void init_display()
{
	int i=0;
	ssd1306ClearScreen();
	ssd1306Refresh();
	ssd1306ClearScreen();
	ssd1306Refresh();

	ssd1306DrawBmp(Pacabot_bmp, 1, 1, 128, 40);
	ssd1306Refresh();

	for (i = 0; i <= 100; i+=5)
	{
	    ssd1306ProgressBar(10, 35, i);
	    //HAL_Delay(1);
	    ssd1306Refresh();
	}
}

void test_Gyro(void)
{
	ADXRS620_Init();
	while(Expander_Joy_State()!=LEFT)
	{
	  ssd1306ClearScreen();
//	  ssd1306PrintInt(10,  25, "Angle = ", (int) gyro_Current_Angle, &Font_5x8);
	  int y = 0;
	  for(int i = 0; i<100; i++)
	  {
//		  y +=  uhADCxConvertedValue_Gyro, &Font_5x8;
		  HAL_Delay(1);
	  }

	  ssd1306PrintInt(10,  35, "Adc   = ",  ((y/100)*3300)/4095, &Font_5x8);

	  ssd1306Refresh();
	}
}

void test_Vbat(void)
{
	  ADC_ChannelConfTypeDef sConfig;
	  int vbat;
	  int vref = 3300;  //mV
	  int coeff = 33;  //%

	  while(Expander_Joy_State()!=LEFT)
	  {
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
	  ssd1306ClearScreen();
	  HAL_ADC_Stop_DMA(&hadc1);
	  sConfig.Channel = ADC_CHANNEL_15;
	  sConfig.Rank = 1;
	  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
	  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&uhADCxConvertedValue_8, 1);

	  vbat = (((vref * uhADCxConvertedValue_8) / 40950) * coeff);

	  ssd1306PrintInt(10,  5, "Vbat =  ", vbat, &Font_5x8);
	  ssd1306Refresh();
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
	  HAL_Delay(100);
	  }

}

void test_Motor_Move() {
	GPIO_InitTypeDef GPIO_InitStruct;
	  TIM_ClockConfigTypeDef sClockSourceConfig;
	  TIM_OC_InitTypeDef sConfigOC;
	  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
	  TIM_MasterConfigTypeDef sMasterConfig;

	  ADXRS620_Init();
	  init_display();

	  uint32_t coef = 6;
	  uint32_t correct = 0;


	  long K1 = 1571;
	  long K2 = 1532;
	  long K3 = 225;
//	  1571
//	  1532
//	  225
//	  23562
//	  22982
//	  225
	  int PWMOld = 0;
	  int PWM =0;
	  int error =0;
	  int errorOld=0;

	  int consigne = 0;




	  htim8.Instance = TIM8;
	  htim8.Init.Prescaler = 4;
	  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim8.Init.Period = 1000-1;
	  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim8.Init.RepetitionCounter = 0;
	  HAL_TIM_Base_Init(&htim8);

	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);

	  HAL_TIM_PWM_Init(&htim8);

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 500;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCNPolarity = TIM_OCPOLARITY_LOW;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
//	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

	  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	  HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);

	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

	  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, SET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);

	  uint32_t Pulses[2] = {50,50};

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);

//	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
//	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
//	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
//	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET);

	  //for(int y = 0; y < 4000; y++)

	  consigne = 200;
	  PWM = consigne;
//	  PWM_R = consigne;
	  while(Expander_Joy_State()!=LEFT)
	  {
		  	      PWMOld = PWM;
		  		  errorOld = error;
		  		  error = (int32_t) gyro_Current_Angle;
		  		  PWM = (K1*error - K2*errorOld + K3*PWMOld)/256;

		if ((((consigne - PWM) + consigne) > 0) && (((PWM - consigne) + consigne) > 0))
		{
	  		  Pulses[0] = (consigne - PWM) + consigne;
	  		  Pulses[1] = (PWM - consigne) + consigne;
		}


//	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
//	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);

	  sConfigOC.Pulse = Pulses[0];
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
	  sConfigOC.Pulse = (1000 - Pulses[1]);
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	  HAL_Delay(1);
	  ssd1306ClearScreen();
	  ssd1306PrintInt(10,  5,  "Correct  = ", error, &Font_5x8);
	  ssd1306PrintInt(10,  15, "Pulses[0]  = ", Pulses[0], &Font_5x8);
	  ssd1306PrintInt(10,  25, "Pulses[1]  = ", Pulses[1], &Font_5x8);
	  ssd1306Refresh();

     }

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, RESET);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, RESET);

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);

	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_4);
//	  HAL_Delay(1);

}

void test_Motors(void)
{
	  TIM_OC_InitTypeDef sConfigOC;
//	  GPIO_InitTypeDef GPIO_InitStruct;

//	  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_8;
//	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	  GPIO_InitStruct.Pull = GPIO_NOPULL;
//	  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
//	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	  HAL_Delay(1000);

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 1000;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

	  sConfigOC.Pulse = 980;
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

	  sConfigOC.Pulse = 900;
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

	  sConfigOC.Pulse = 1000;
	  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4);

	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, SET);

	  HAL_Delay(5000);
	  while(Expander_Joy_State()!=LEFT)
	  {

	  }

	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, RESET);
}

void test_Encoders(void)
{

	  Encoders_Init();
	  while(Expander_Joy_State()!=LEFT)
	  {
		  ssd1306ClearScreen();
//		  ssd1306PrintInt(0, 6, "REV = ", toto, &Font_3x6);
//		  ssd1306PrintInt(0, 13, "CNT = ",  (&htim1)->Instance->CNT, &Font_3x6);
		  ssd1306PrintInt(0, 7, "L_REV =  ", left_encoder.nb_revolutions, &Font_3x6);
		  ssd1306PrintInt(0, 14, "L_CNT =  ",  __HAL_TIM_GetCounter(&htim1), &Font_3x6);
		  ssd1306PrintInt(0, 21, "L_DIR =  ",  __HAL_TIM_DIRECTION_STATUS(&htim1), &Font_3x6);

		  ssd1306PrintInt(0, 35, "R_REV =  ", right_encoder.nb_revolutions, &Font_3x6);
		  ssd1306PrintInt(0, 42, "R_CNT =  ",  __HAL_TIM_GetCounter(&htim3), &Font_3x6);
		  ssd1306PrintInt(0, 49, "R_DIR =  ",  __HAL_TIM_DIRECTION_STATUS(&htim3), &Font_3x6);
		  ssd1306Refresh();
		  HAL_Delay(10);
	  }
}

void test_Beeper(void)
{
	  if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Starting Error */
	//    Error_Handler();
	  }
	  HAL_Delay(100);
	  if(HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Starting Error */
	//    Error_Handler();
	  }
	  HAL_Delay(100);
	  if(HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Starting Error */
	//    Error_Handler();
	  }
	  HAL_Delay(200);
	  if(HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4) != HAL_OK)
	  {
	    /* Starting Error */
	//    Error_Handler();
	  }
}

void test_Expander(void)
{
	  ExpanderReset();
	  HAL_Delay(50);
	  ExpanderSetbit(4,0);
	  HAL_Delay(50);
	  ExpanderSetbit(4,1);
	  HAL_Delay(50);
	  ExpanderSetbit(4,0);
	  HAL_Delay(50);
	  ExpanderSetbit(5,0);
	  HAL_Delay(50);
	  ExpanderSetbit(5,1);
	  HAL_Delay(50);
	  ExpanderSetbit(5,0);
	  HAL_Delay(50);
	  ExpanderSetbit(6,0);
	  HAL_Delay(50);
	  ExpanderSetbit(6,1);
	  HAL_Delay(50);
	  ExpanderSetbit(6,0);
	  HAL_Delay(50);
	  ExpanderSetbit(4,1);
	  HAL_Delay(50);
	  ExpanderSetbit(4,0);
	  HAL_Delay(50);
	  ExpanderSetbit(4,1);
	  HAL_Delay(50);
	  ExpanderSetbit(5,1);
	  HAL_Delay(50);
	  ExpanderSetbit(5,0);
	  HAL_Delay(50);
	  ExpanderSetbit(5,1);
	  HAL_Delay(50);
	  ExpanderSetbit(6,1);
	  HAL_Delay(50);
	  ExpanderSetbit(6,0);
	  HAL_Delay(50);
	  ExpanderSetbit(6,1);
}

void test_PWled(void)
{
	int i;
	  /* Infinite loop */
	  for(i = 0; i<1000; i++)
	  {
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, SET);
	      HAL_Delay(1000);
	      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, RESET);
	      HAL_Delay(1000);
	  }
}

void test_Oled(void)
{
  int i;
  ssd1306ClearScreen();
  ssd1306DrawBmp(Pacabot_bmp, 1, 10, 128, 40);
  ssd1306Refresh();
  HAL_Delay(3500);
  ssd1306ClearScreen();
  ssd1306Refresh();
  // miniature bitmap display
  ssd1306DrawCircle(40, 30, 20);
//  ssd1306DrawCircle(50, 20, 10);
  ssd1306FillCircle(100, 40, 15);
  ssd1306DrawRect(110, 3, 15, 6);
  ssd1306FillRect(1, 60, 10, 20);
  ssd1306DrawLine(5, 45, 70, 60);
  ssd1306Refresh();
  HAL_Delay(5500);
  ssd1306ClearScreen();

  for (i = 0; i <= 100; i+=2)
  {
      ssd1306ProgressBar(10, 20, i);
//      HAL_Delay(1);
      ssd1306Refresh();
  }

  ssd1306ShiftFrameBuffer(8);
  ssd1306DrawString(13, 1, "Oled 128x64", &Font_8x8); // 3x6 is UPPER CASE only
  ssd1306Refresh();
  HAL_Delay(1500);
  ssd1306DrawString(1, 25, "Driver for STM32f4xx", &Font_5x8); // 3x6 is UPPER CASE only
  ssd1306Refresh();
  HAL_Delay(500);
  ssd1306DrawString(1, 35, "2 Wire SPI mode", &Font_5x8); // 3x6 is UPPER CASE only
  ssd1306Refresh();
  HAL_Delay(1500);
  ssd1306DrawString(10, 55, "BY PLF, PACABOT TEAM", &Font_3x6); // 3x6 is UPPER CASE only
  ssd1306Refresh();
  HAL_Delay(5000);

  ssd1306ClearScreen();
  ssd1306Refresh();
}

void test_Telemeters(void)
{
	  Telemeters_Init();
	  Telemeters_Start();
	  while(1)
	  {
		  ssd1306ClearScreen();
		  ssd1306PrintInt(10, 5,  "LFRONT  = ", (uint16_t) telemeters.left_front.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 15, "RFRONT  = ", (uint16_t) telemeters.right_front.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 25, "LDIAG   = ", (uint16_t) telemeters.left_diag.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 35, "RDIAG   = ", (uint16_t) telemeters.right_diag.adc_value, &Font_5x8);
		  ssd1306Refresh();
	  }
}

void test_LineSensors(void)
{

	  LineSensors_Init();
	  LineSensors_Start();

//	  LineSensors_Init();
//	  LineSensors_Start();
	  while(Expander_Joy_State()!=LEFT)
	  {
		  ssd1306ClearScreen();
		  ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 25, "FRONT     =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
		  ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);
		  ssd1306Refresh();
	  }

}
long _pow(long x, long y)
{
	return pow(x,y);
}
