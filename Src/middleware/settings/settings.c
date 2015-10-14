/*
 * settings.c
 *
 *  Created on: 3 avr. 2015
 *      Author: Colin
 */
#include "middleware/settings/settings.h"
#include "peripherals/eeprom/24lc64.h"
#include "config/basetypes.h"

#include "gpio.h"

int settingsInit (void)
{
	zhonxSettings.calibration_enabled=false;
	zhonxSettings.color_sensor_enabled=false;
	zhonxSettings.maze_end_coordinate.x=1;
	zhonxSettings.maze_end_coordinate.y=1;
	zhonxSettings.sleep_delay_s=300; // the robot will go sleep in zhonxSettings.sleep_delay_s S
	return SETTING_MODULE_E_SUCCESS;
}

// Shutdown the robot
void halt(void)
{
    GPIO_InitTypeDef gpio;

    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
}
