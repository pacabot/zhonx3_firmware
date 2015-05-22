/*
 * settings.c
 *
 *  Created on: 3 avr. 2015
 *      Author: Colin
 */
#include "middleware/settings/settings.h"
#include "peripherals/eeprom/24lc64.h"
#include "config/basetypes.h"
int settingsInit (void)
{
	zhonxSettings.calibration_enabled=false;
	zhonxSettings.color_sensor_enabled=false;
	zhonxSettings.x_finish_maze=1;
	zhonxSettings.y_finish_maze=1;
	return SETTING_MODULE_E_SUCCESS;
}
