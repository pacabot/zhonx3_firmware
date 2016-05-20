/*
 * settings.c
 *
 *  Created on: 3 avr. 2015
 *      Author: Colin
 */

#include "config/basetypes.h"
#include "config/config.h"
#include "middleware/settings/settings.h"
#include "peripherals/eeprom/24lc64.h"

#include "gpio.h"

settings zhonxSettings;

CALIBRATION_DATA *zhonxCalib_data = (CALIBRATION_DATA *)CALIBRATION_DATA_ADDR;

int settingsInit(void)
{
	zhonxSettings.calibration_enabled=FALSE;
	zhonxSettings.nime_competition=FALSE;
	zhonxSettings.return_to_start_cell = FALSE;
	zhonxSettings.maze_end_coordinate.x=7;
	zhonxSettings.maze_end_coordinate.y=7;
	zhonxSettings.sleep_delay_s=300; // the robot will go sleep in zhonxSettings.sleep_delay_s S
	zhonxSettings.wall_know_cost = 1;
	zhonxSettings.cell_cost = 5;
	zhonxSettings.start_orientation = 0;
    zhonxSettings.speeds_scan.max_speed_rotation = SCAN_SPEED_ROTATION;
    zhonxSettings.speeds_scan.max_speed_traslation = SCAN_MAX_SPEED_TRANSLATION;
    zhonxSettings.speeds_scan.min_speed = SCAN_MIN_SPEED_TRANSLATION;
    zhonxSettings.speeds_run1.max_speed_rotation = RUN1_SPEED_ROTATION;
    zhonxSettings.speeds_run1.max_speed_traslation = RUN1_MAX_SPEED_TRANSLATION;
    zhonxSettings.speeds_run1.min_speed = RUN1_MIN_SPEED_TRANSLATION;
    zhonxSettings.speeds_run2.max_speed_rotation = RUN2_SPEED_ROTATION;
    zhonxSettings.speeds_run2.max_speed_traslation = RUN2_MAX_SPEED_TRANSLATION;
    zhonxSettings.speeds_run2.min_speed = RUN2_MIN_SPEED_TRANSLATION;

    // Flash Initialization
    // TODO: Check returned values of the following functions
    flash_init();
    flash_open(NULL /* XXX: Not used */, &zhonxSettings.h_flash);

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
