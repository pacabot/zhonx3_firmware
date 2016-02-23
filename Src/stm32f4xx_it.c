/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN 0 */

#include "config/config.h"
#include "config/basetypes.h"

#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"
#include "middleware/ring_buffer/ring_buffer.h"
#include "middleware/cmdline/cmdline_parser.h"

// Buffer used for incoming commands thru serial port
char  serial_buffer[1024];

static inline void parseReceivedByte(void);

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles RCC global interrupt.
*/
void RCC_IRQHandler(void)
{
  /* USER CODE BEGIN RCC_IRQn 0 */

  /* USER CODE END RCC_IRQn 0 */
  /* USER CODE BEGIN RCC_IRQn 1 */

  /* USER CODE END RCC_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream1 global interrupt.
*/
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream3 global interrupt.
*/
void DMA1_Stream3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

  /* USER CODE END DMA1_Stream3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
  /* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

  /* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream5 global interrupt.
*/
void DMA1_Stream5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

  /* USER CODE END DMA1_Stream5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
  /* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

  /* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
* @brief This function handles DMA1 stream6 global interrupt.
*/
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */

  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
* @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
*/
void ADC_IRQHandler(void)
{
  /* USER CODE BEGIN ADC_IRQn 0 */

  /* USER CODE END ADC_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  HAL_ADC_IRQHandler(&hadc3);
  /* USER CODE BEGIN ADC_IRQn 1 */

  /* USER CODE END ADC_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
*/
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles TIM3 global interrupt.
*/
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
* @brief This function handles TIM4 global interrupt.
*/
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  uint32_t      uart_status_flag;
  uint32_t      uart_it_flag;

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */

  // Get UART flags
  uart_status_flag = __HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE);
  uart_it_flag = __HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_RXNE);

  if ((uart_status_flag != RESET) && (uart_it_flag != RESET))
  {
      // Clear interrupt flag
      __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);

      // Call the relevant function
      parseReceivedByte();
  }

  /* USER CODE END USART3_IRQn 1 */
}

/**
* @brief This function handles TIM5 global interrupt.
*/
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
* @brief This function handles TIM6 global interrupt, DAC1 and DAC2 underrun error interrupts.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
* @brief This function handles TIM7 global interrupt.
*/
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
* @brief This function handles DMA2 stream0 global interrupt.
*/
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/* USER CODE BEGIN 1 */

#ifdef CONFIG_USE_CMDLINE
// Command Line mode enabled
static inline void parseReceivedByte(void)
{
    static char   *pBuffer = serial_buffer;
    unsigned char c;

    // Get the character received
    c = (uint16_t) (huart3.Instance->DR & (uint16_t) 0x01FF);

    if (cmdline_ctxt.is_initialized == FALSE)
    {
        return;
    }

    // Command Line mode is enabled
    switch (c)
    {
        case CMDLINE_CR:
            // Carriage Return
            cmdline_ctxt.cmd_len = (pBuffer - serial_buffer) + 1;
            cmdline_ctxt.cmd_received = TRUE;
            *pBuffer = c;
            pBuffer = serial_buffer;
            return;

        case CMDLINE_LF:
            // Line Feed
            pBuffer = serial_buffer;
            break;

        case CMDLINE_BS:
            // Backspace
            if (pBuffer == serial_buffer)
            {
//                HAL_UART_Transmit(&huart3, (unsigned char *) "\x07", 1,
//                        10);
            }
            else
            {
//                HAL_UART_Transmit(&huart3, (unsigned char *) "\x08\x7F",
//                        2, 20);
                pBuffer--;
            }
            break;

        default:
            *pBuffer = c;
            pBuffer++;
            break;
    }
}

#else

// States used by the state machine
#define CMD_STATE_WAIT_STX              0
#define CMD_STATE_WAIT_LEN_MSB          1
#define CMD_STATE_WAIT_LEN_LSB          2
#define CMD_STATE_WAIT_INSTRUCTION      3
#define CMD_STATE_WAIT_DATA             4

#define CMD_STX                 0x02
#define CMD_ETX                 0x03

// Hexadecimal Command mode enabled
static inline void parseReceivedByte(void)
{
    static char   *pBuffer = serial_buffer;
    unsigned char c;
    static int    catch_BT_events = TRUE;
    static int    cmd_state = CMD_STATE_WAIT_STX;
    static int    cmd_len = 0;

    // Get the character received
    c = (uint16_t) (huart3.Instance->DR & (uint16_t) 0x01FF);

    // Bluetooth events manager
    if (catch_BT_events == TRUE)
    {
        switch (c)
        {
            case CMDLINE_CR:
                // Carriage Return
                cmdline_ctxt.cmd_len = (pBuffer - serial_buffer) + 1;
                cmdline_ctxt.cmd_received = TRUE;
                *pBuffer = c;
                pBuffer = serial_buffer;
                return;

            case CMDLINE_LF:
                // Line Feed
                pBuffer = serial_buffer;
                break;

            default:
                *pBuffer = c;
                pBuffer++;
                break;
        }
    }

    // State machine
    switch(cmd_state)
    {
        // Receive STX
        case CMD_STATE_WAIT_STX:
            if (c == CMD_STX)
            {
                cmd_state = CMD_STATE_WAIT_LEN_MSB;
                // Disable bluetooth events manager
                catch_BT_events = FALSE;
            }
            break;

        // Receive length - MSB
        case CMD_STATE_WAIT_LEN_MSB:
            *pBuffer = c;
            pBuffer++;
            cmd_len = (c << 8);
            cmd_state = CMD_STATE_WAIT_LEN_LSB;
            break;

        // Receive length - LSB
        case CMD_STATE_WAIT_LEN_LSB:
            *pBuffer = c;
            pBuffer++;
            cmd_len |= c;
            cmd_state = CMD_STATE_WAIT_INSTRUCTION;
            break;

        // Receive instruction
        case CMD_STATE_WAIT_INSTRUCTION:
            *pBuffer = c;
            pBuffer++;
            cmd_state = CMD_STATE_WAIT_DATA;
            break;

        // Receive data
        case CMD_STATE_WAIT_DATA:
            *pBuffer = c;
            pBuffer++;
            cmd_len--;
            if (cmd_len == 0)
            {
                // TODO: Set command received flag

                // Enable bluetooth events manager
                catch_BT_events = TRUE;
                // Reset state machine
                cmd_state = CMD_STATE_WAIT_STX;
            }
            break;
    }
}
#endif

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
