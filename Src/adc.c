/**
  ******************************************************************************
  * File Name          : ADC.c
  * Date               : 19/02/2015 00:48:17
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 6;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_14;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T5_TRGO;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = ENABLE;
  hadc2.Init.NbrOfDiscConversion = 4;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 4;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 0;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = 2;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_InjectionConfTypeDef sConfigInjected;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION12b;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = ENABLE;
  hadc3.Init.NbrOfDiscConversion = 3;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = EOC_SEQ_CONV;
  HAL_ADC_Init(&hadc3);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 0;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 3;
  HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

    /**Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 2;
  HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected);

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* Peripheral clock enable */
    __ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PA7     ------> ADC1_IN7
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Peripheral DMA init*/
  
    hdma_adc1.Instance = DMA2_Stream0;
    hdma_adc1.Init.Channel = DMA_CHANNEL_0;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_adc1);

    __HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* Peripheral clock enable */
    __ADC2_CLK_ENABLE();
  
    /**ADC2 GPIO Configuration    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PC3     ------> ADC2_IN13
    PA4     ------> ADC2_IN4
    PA5     ------> ADC2_IN5
    PA6     ------> ADC2_IN6 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* Peripheral clock enable */
    __ADC3_CLK_ENABLE();
  
    /**ADC3 GPIO Configuration    
    PC2     ------> ADC3_IN12
    PA1     ------> ADC3_IN1
    PA3     ------> ADC3_IN3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(ADC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc)
{

  if(hadc->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __ADC1_CLK_DISABLE();
  
    /**ADC1 GPIO Configuration    
    PA7     ------> ADC1_IN7
    PC4     ------> ADC1_IN14
    PC5     ------> ADC1_IN15 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    /* Peripheral DMA DeInit*/
    HAL_DMA_DeInit(hadc->DMA_Handle);

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __ADC2_CLK_DISABLE();
  
    /**ADC2 GPIO Configuration    
    PC0     ------> ADC2_IN10
    PC1     ------> ADC2_IN11
    PC3     ------> ADC2_IN13
    PA4     ------> ADC2_IN4
    PA5     ------> ADC2_IN5
    PA6     ------> ADC2_IN6 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_3);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(hadc->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __ADC3_CLK_DISABLE();
  
    /**ADC3 GPIO Configuration    
    PC2     ------> ADC3_IN12
    PA1     ------> ADC3_IN1
    PA3     ------> ADC3_IN3 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_3);

    /* Peripheral interrupt Deinit*/
  /* USER CODE BEGIN ADC3:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC3:ADC_IRQn disable */

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
