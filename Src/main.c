
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#include <application/display/menu.h>
#include <middleware/cmdline/cmdline_parser.h>
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/positionControl.h>
#include <middleware/moves/mazeMoves/spyPost.h>
#include <middleware/settings/settings.h>
#include <peripherals/bluetooth/bluetooth.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/flash/flash.h>
#include <peripherals/multimeter/multimeter.h>
#include <peripherals/times_base/times_base.h>
#include <peripherals/tone/tone.h>
#include <peripherals/telemeters/telemeters.h>
#include <peripherals/motors/motors.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static int setZhonxName(void);

static char zhonxName[24];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
static inline void blockingPrintf(const char *format, ...);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern const menuItem mainMenu;
extern const menuItem zhonxNameMenu;

static inline void blockingPrintf(const char *format, ...)
{
    bluetoothPrintf(format);
    bluetoothWaitReady();
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    CMDLINE_CONTEXT cmd_context;
    const char *zhonx_info = (char *)CONFIG_ZHONX_INFO_ADDR;
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_I2C1_Init();
    MX_RNG_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
    MX_TIM7_Init();
    MX_TIM8_Init();
    MX_TIM11_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    expanderInit();
    ssd1306Init(0);
    mainControlInit();
    timesBaseInit();
    ledPowerBlink(990, 10);
    settingsInit();
    mulimeterInit();
    bluetoothInit();
    toneInit();
    spyPostInit();

    for (int i = 0; i < 6; i++)
    {
        int j = 0b11100;
        expanderSetLeds(j >> i);
        HAL_Delay(20);
    }

    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);

    toneSetVolulme(100);
    tone(F4, 50);
    toneItMode(A4, 50);

    // Register Output callback
    cmd_context.out = blockingPrintf;
    // Initialize Command Line module
    cmdline_init(&cmd_context);

    // Check if robot name is populated in Flash
    memset(zhonxName, 0, sizeof(zhonxName));
    // Retrieve ZHONX information from flash if any
    if (zhonx_info[0] == 'Z')
    {
        strcpy(zhonxName, zhonx_info);
    }

    while (zhonxName[0] == 0)
    {
        menu(zhonxNameMenu);
    }

    while (1)
    {
        menu(mainMenu);
        powerOffConfirmation();
    }

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
        ;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
     */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 16;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time 
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
int setMeddle(void)
{
    strcpy(zhonxName, ZHONX_GENERATION);
    strcat(zhonxName, " MEDDLE");
    // Set Bluetooth device name
    bluetoothCmd("at+ab config DeviceName=Meddle");
    bluetoothCmd("AT+AB Reset");
    return setZhonxName();
}

int setDark(void)
{
    strcpy(zhonxName, ZHONX_GENERATION);
    strcat(zhonxName, " DARK");
    // Set Bluetooth device name
    bluetoothCmd("at+ab config DeviceName=Dark");
    bluetoothCmd("AT+AB Reset");
    return setZhonxName();
}

static int setZhonxName(void)
{
    int rv;

    // Write Name in flash
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)CONFIG_ZHONX_INFO_ADDR,
                     (unsigned char *)zhonxName, strlen(zhonxName) + 1);
    if (rv != FLASH_E_SUCCESS)
    {
        // TODO: handle this error
    }
    return 0;
}

void HardFault_Handler(void)
{
    telemetersStop();
    motorsDriverSleep(ON);
    motorsBrake();
    tone(A4, 4000);
    halt();
}

//void MemManage_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(C4, 4000);
//    halt();
//}
//
//void BusFault_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(C4, 4000);
//    halt();
//}
//
//void UsageFault_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(D4, 4000);
//    halt();
//}
//
//void SVC_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(E4, 4000);
//    halt();
//}
//
//void DebugMon_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(F4, 4000);
//    halt();
//}
//
//void PendSV_Handler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(G4, 4000);
//    halt();
//}
//
//void FLASH_IRQHandler()
//{
//    telemetersStop();
//    motorsDriverSleep(ON);
//    motorsBrake();
//    tone(A5, 4000);
//    halt();
//}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
