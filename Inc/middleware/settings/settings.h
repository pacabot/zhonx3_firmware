/*
 * setting.h
 *
 *  Created on: 3 avr. 2015
 *      Author: Colin
 */

#ifndef SETTING_H_
#define SETTING_H_

/* dependencies */
#include "arm_math.h"
#include "config/module_id.h"
#include "peripherals/flash/flash.h"
#include "config/basetypes.h"
#include "middleware/moves/mazeMoves/spyPost.h"
#include "middleware/moves/mazeMoves/spyWall.h"
#include "middleware/controls/pidController/pidCalculator.h"
#include "peripherals/gyroscope/adxrs620.h"


/*********************** ZHONX generation and version *********************************/
#define ZHONX_GENERATION          "Z3"

/* Error codes */
#define SETTING_MODULE_E_SUCCESS	0
#define SETTING_MODULE_E_ERROR	MAKE_ERROR(SETTIING_MODULE_ID, 1)

/* memory address */

/* structure of settings */
typedef struct
{
    int x;
    int y;
} coordinate;

typedef struct
{
    int min_speed;
    int max_speed_traslation;
    int max_speed_rotation;
} speed_settings;

typedef struct
{
	char calibration_enabled;
	char nime_competition;
	coordinate maze_end_coordinate;
	int wall_know_cost;
	int cell_cost;
	int start_orientation;
	unsigned int sleep_delay_s;
	char return_to_start_cell;
    speed_settings speeds_scan;
    speed_settings speeds_run1;
    speed_settings speeds_run2;
	FLASH_HANDLE h_flash;
} settings;

typedef struct
{
    spyPostRefProfileStruct spyPost[2];
    spyWallCalibStruct      spyWall;
    gyro_calib_struct       gyro;
    arm_pid_instance_f32    pid_encoders;
    arm_pid_instance_f32    pid_gyro;
    arm_pid_instance_f32    pid_telemeters;
    arm_pid_instance_f32    pid_lineFollow;
} CALIBRATION_DATA;

extern settings zhonxSettings;

// Flash stored data
extern CALIBRATION_DATA *zhonxCalib_data;

/* Exported Bluetooth parameters */
extern presetParam BTpresetBaudRate;

int settingsInit(void);
#endif /* SETTING_H_ */
