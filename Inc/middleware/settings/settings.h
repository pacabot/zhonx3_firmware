/*
 * setting.h
 *
 *  Created on: 3 avr. 2015
 *      Author: Colin
 */

#ifndef SETTING_H_
#define SETTING_H_

/* dependencies */
#include "config/module_id.h"
#include "peripherals/flash/flash.h"

/*********************** ZHONX generation and version *********************************/
#define ZHONX_GENERATION          "Z3"

/* Error codes */
#define SETTING_MODULE_E_SUCCESS	0
#define SETTING_MODULE_E_ERROR	MAKE_ERROR(SETTIING_MODULE_ID, 1)

/* memory address */

/* structure of settings */
typedef struct coordinate
{
  int x;
  int y;
}coordinate;

typedef struct
{
	char calibration_enabled;
	char nime_competition;
	coordinate maze_end_coordinate;
	unsigned int sleep_delay_s;
	unsigned long threshold_color;
	unsigned long threshold_greater;
	FLASH_HANDLE h_flash;
}settings;

extern settings zhonxSettings;

/* Exported Bluetooth parameters */
extern presetParam BTpresetBaudRate;

int settingsInit (void);
void halt(void);
#endif /* SETTING_H_ */
