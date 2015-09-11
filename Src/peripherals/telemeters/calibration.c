/*
 * calibration.c
 *
 *  Created on: 15 août 2015
 *      Author: Colin
 */

#include "stdbool.h"
#include "config/basetypes.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/positionControl.h"

#include "peripherals/telemeters/telemeters.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/bluetooth/bluetooth.h"

#ifdef DARK
int telemeter_FR_profile[NUMBER_OF_CELL];
int telemeter_FL_profile[NUMBER_OF_CELL];
int telemeter_DR_profile[NUMBER_OF_CELL];
int telemeter_DL_profile[NUMBER_OF_CELL];
#else //MEDDLE
int telemeter_FR_profile[NUMBER_OF_CELL];
int telemeter_FL_profile[NUMBER_OF_CELL];
int telemeter_DR_profile[NUMBER_OF_CELL];
int telemeter_DL_profile[NUMBER_OF_CELL];
#endif
extern telemetersStruct telemeters;
int telemeterFrontCalibration ()
{
	int cell_left = 0;
	int cell_right = 0;

	memset(telemeter_FL_profile,0,sizeof(int)*NUMBER_OF_CELL);
	memset(telemeter_FR_profile,0,sizeof(int)*NUMBER_OF_CELL);
	telemetersInit();
	telemetersStart();
	mainControlInit();

	control_params.wall_follow_state = FALSE;
	position_control.position_type = ENCODERS; // maybe gyro
	move(0,0,0,0);
	cell_left = telemeters.FL.avrg / STEP;
	cell_right = telemeters.FR.avrg / STEP;

	move(0,-DISTANCE_CALIBRATED,3,0);
	ssd1306Printf(0,0,&Font_5x8,"debut");
	ssd1306Refresh();

	while(isEndMove() != true)
	{
		if ((telemeters.FL.avrg / STEP) > cell_left)
		{
			cell_left = (telemeters.FL.avrg / STEP);
			telemeter_FL_profile[cell_left] = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder))/2;
		}
		if ((telemeters.FR.avrg / STEP) > cell_right)
		{
			cell_right = (telemeters.FR.avrg / STEP);
			telemeter_FR_profile[cell_right] = (encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder))/2;
		}
	}
	bluetoothPrintf("front value\r\n");
	for (int i=0; i<NUMBER_OF_CELL; i++)
	{
		bluetoothPrintf("left %5d right %5d\r\n",telemeter_FL_profile[i],telemeter_FR_profile[i]);
	}
	return CALIBATION_E_SUCCESS;
}

int telemeterDiagCalibration ()
{
	int cell_left = 0;
	int cell_right = 0;

	memset(telemeter_FL_profile,0,sizeof(int)*NUMBER_OF_CELL);
	memset(telemeter_FR_profile,0,sizeof(int)*NUMBER_OF_CELL);
	telemetersInit();
	telemetersStart();
	mainControlInit();

	control_params.wall_follow_state = FALSE;
	position_control.position_type = ENCODERS; // maybe gyro
	move(0,0,0,0);
	cell_left = telemeters.FL.avrg / STEP;
	cell_right = telemeters.FR.avrg / STEP;

	move(0,-(DISTANCE_CALIBRATED)*cosf(45),3,0);
	ssd1306Printf(0,0,&Font_5x8,"debut");
	ssd1306Refresh();

	while(isEndMove() != true)
	{
		if ((telemeters.DL.avrg / STEP) > cell_left)
		{
			cell_left = (telemeters.DL.avrg / STEP);
			telemeter_FL_profile[cell_left] = ((encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder))/2)*cosf(45);
		}
		if ((telemeters.DR.avrg / STEP) > cell_right)
		{
			cell_right = (telemeters.DR.avrg / STEP);
			telemeter_FR_profile[cell_right] = ((encoderGetDistance(&left_encoder) + encoderGetDistance(&right_encoder))/2)*cosf(45);
		}
	}
	bluetoothPrintf("diag value\r\n");
	for (int i=0; i<NUMBER_OF_CELL; i++)
	{
		bluetoothPrintf("left %5d right %5d\r\n",telemeter_FL_profile[i],telemeter_FR_profile[i]);
	}
	return CALIBATION_E_SUCCESS;
}

