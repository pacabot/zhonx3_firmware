/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define R_FRONT_RX_Pin GPIO_PIN_1
#define R_FRONT_RX_GPIO_Port GPIOC
#define LINE_SENSOR_R2_Pin GPIO_PIN_2
#define LINE_SENSOR_R2_GPIO_Port GPIOC
#define LINE_SENSOR_R1_Pin GPIO_PIN_3
#define LINE_SENSOR_R1_GPIO_Port GPIOC
#define LINE_SENSOR_F_Pin GPIO_PIN_1
#define LINE_SENSOR_F_GPIO_Port GPIOA
#define EN_LINE_SENSORS_Pin GPIO_PIN_2
#define EN_LINE_SENSORS_GPIO_Port GPIOA
#define LINE_SENSOR_L2_Pin GPIO_PIN_3
#define LINE_SENSOR_L2_GPIO_Port GPIOA
#define LINE_SENSOR_L1_Pin GPIO_PIN_4
#define LINE_SENSOR_L1_GPIO_Port GPIOA
#define L_DIAG_RX_Pin GPIO_PIN_5
#define L_DIAG_RX_GPIO_Port GPIOA
#define L_FRONT_RX_Pin GPIO_PIN_6
#define L_FRONT_RX_GPIO_Port GPIOA
#define GYRO_TEMP_Pin GPIO_PIN_7
#define GYRO_TEMP_GPIO_Port GPIOA
#define GYRO_RATE_Pin GPIO_PIN_4
#define GYRO_RATE_GPIO_Port GPIOC
#define VBAT_Pin GPIO_PIN_5
#define VBAT_GPIO_Port GPIOC
#define GET_ADC_BAT_Pin GPIO_PIN_0
#define GET_ADC_BAT_GPIO_Port GPIOB
#define L_FRONT_TX_Pin GPIO_PIN_1
#define L_FRONT_TX_GPIO_Port GPIOB
#define R_FRONT_TX_Pin GPIO_PIN_10
#define R_FRONT_TX_GPIO_Port GPIOB
#define DIAG_TX_Pin GPIO_PIN_11
#define DIAG_TX_GPIO_Port GPIOB
#define L_MOTOR_IN1_Pin GPIO_PIN_6
#define L_MOTOR_IN1_GPIO_Port GPIOC
#define L_MOTOR_IN2_Pin GPIO_PIN_7
#define L_MOTOR_IN2_GPIO_Port GPIOC
#define R_MOTOR_IN1_Pin GPIO_PIN_8
#define R_MOTOR_IN1_GPIO_Port GPIOC
#define R_MOTOR_IN2_Pin GPIO_PIN_9
#define R_MOTOR_IN2_GPIO_Port GPIOC
#define L_ENCODER_A_Pin GPIO_PIN_8
#define L_ENCODER_A_GPIO_Port GPIOA
#define L_ENCODER_B_Pin GPIO_PIN_9
#define L_ENCODER_B_GPIO_Port GPIOA
#define MOTORS_FL_Pin GPIO_PIN_10
#define MOTORS_FL_GPIO_Port GPIOA
#define MOTORS_SLEEP_Pin GPIO_PIN_11
#define MOTORS_SLEEP_GPIO_Port GPIOA
#define GPIO_BASE_Pin GPIO_PIN_12
#define GPIO_BASE_GPIO_Port GPIOA
#define PW_LED_Pin GPIO_PIN_15
#define PW_LED_GPIO_Port GPIOA
#define WP_EEPROM_Pin GPIO_PIN_3
#define WP_EEPROM_GPIO_Port GPIOB
#define R_ENCODER_A_Pin GPIO_PIN_4
#define R_ENCODER_A_GPIO_Port GPIOB
#define R_ENCODER_B_Pin GPIO_PIN_5
#define R_ENCODER_B_GPIO_Port GPIOB
#define PW_KILL_Pin GPIO_PIN_8
#define PW_KILL_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
