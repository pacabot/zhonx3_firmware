/*
 * @file line_sensors.c
 * @date 8 may 2018
 * @Author:Colin Roubaud
 * @brief this file contain the necessary for follow a line
 */

/* Standard includes*/
#include <string.h>
#include "config/basetypes.h"
#include "stm32f4xx_hal.h"

/* application includes */
#include "application/statistiques/statistiques.h"

/* middleware includes */
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/moves/basicMoves/basicMoves.h"

/* peripheral includes */
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/tone/tone.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/motors/motors.h"
#include "peripherals/expander/pcf8574.h"

/*include for this module*/
#include "middleware/line_sensors/line_sensors.h"
FloorSensorsCalib current_floor_calib;
FloorSensorsCalib floorSensorCalib(void)
{
	FloorSensorsCalib rv;
	struct FloorSensorsValues current;
	lineSensorsInit();

	positionControlSetPositionType(ENCODERS);
	mainControlSetFollowType(NO_FOLLOW);

	HAL_Delay(1000);

//	double cdg=0;
//	double cdg2=0;


	lineSensorsStart();

	tone(e, 500);
	basicMove(CALIB_ANGL / 2, 0, 100, 0);
	while(hasMoveEnded() != TRUE);
	basicMove(-CALIB_ANGL, 0, 150, 0);
	// -------------------------------------------------------------
	// Init line Sensor

	rv.max[0] = getLineSensorAdc(LINESENSOR_EXT_L);
	rv.max[1] = getLineSensorAdc(LINESENSOR_L);
	rv.max[2] = getLineSensorAdc(LINESENSOR_F);
	rv.max[3] = getLineSensorAdc(LINESENSOR_R);
	rv.max[4] = getLineSensorAdc(LINESENSOR_EXT_R);
	memcpy(rv.min, rv.max, sizeof(rv.max));

	while(hasMoveEnded() != TRUE)
	{
		current.val[0] = getLineSensorAdc(LINESENSOR_EXT_L);
		current.val[1] = getLineSensorAdc(LINESENSOR_L);
		current.val[2] = getLineSensorAdc(LINESENSOR_F);
		current.val[3] = getLineSensorAdc(LINESENSOR_R);
		current.val[4] = getLineSensorAdc(LINESENSOR_EXT_R);

		for (int i = 0; i < FLOOR_SENSOR_CNT; ++i) {
			if(current.val[i] < rv.min[i])
			{
				rv.min[i] = current.val[i];
			}
			if(current.val[i] > rv.max[i])
			{
				rv.max[i] = current.val[i];
			}
		}
	}

	if(cAverage(current.val, FLOOR_SENSOR_CNT) > MAX_FLOOR_SENSORS_VAL / 2)
	{
		int tmp [FLOOR_SENSOR_CNT];
		memcpy(tmp, rv.max, sizeof(rv.max));
		memcpy(rv.max, rv.min, sizeof(rv.max));
		memcpy(rv.min, tmp, sizeof(rv.max));
	}
	tone(b, 500);
	tone(c, 500);

	basicMove(CALIB_ANGL / 2, 0, 30, 30);
	motorsDriverSleep(ON);

	int joystick = expanderJoyFiltered();
	while (joystick!=JOY_LEFT)
	{

		joystick = expanderJoyFiltered();

		// todo : calculate line position for print it in cdg

		ssd1306ClearScreen(MAIN_AREA);
//		ssd1306PrintfAtLine(10, 1, &Font_5x8, "Centre =  %d", cdg);
		ssd1306Refresh();

	}
	lineSensorsStop();
	current_floor_calib = rv; //store the calib for use it
	return rv;
}

struct FloorSensorsValues getFloorSensorsValues(FloorSensorsCalib calib)
{
	struct FloorSensorsValues rv;
	double tmp[FLOOR_SENSOR_CNT];

	tmp[0] = getLineSensorAdc(LINESENSOR_EXT_L);
	tmp[1] = getLineSensorAdc(LINESENSOR_L);
	tmp[2] = getLineSensorAdc(LINESENSOR_F);
	tmp[3] = getLineSensorAdc(LINESENSOR_R);
	tmp[4] = getLineSensorAdc(LINESENSOR_EXT_R);
	for (int i = 0; i < FLOOR_SENSOR_CNT; ++i)
	{
		rv.val[i] = (int) (tmp[i] - calib.min[i]) * (MAX_FLOOR_SENSORS_VAL) / (calib.max[i] - calib.min[i]);
	}
	return rv;
}
int getLinePos(void)
{
	struct FloorSensorsValues sensors = getFloorSensorsValues(current_floor_calib);
	int sensor_relativ_pos;
	int pos;
	int max_i = 2;
	int max = sensors.val[max_i];
	for (int i = 0; i < FLOOR_SENSOR_CNT; ++i)
	{
		if(max < sensors.val[i])
		{
			max = sensors.val[i];
			max_i = i;
		}
	}

	if(max_i == 0) // the line is largely on the left of the robot
	{
		sensor_relativ_pos = -sensors.val[max_i];
	}
	else if(max_i == (FLOOR_SENSOR_CNT - 1)) // the line is largely on the right of the robot
	{
		sensor_relativ_pos = sensors.val[max_i];
	}
	// so the line is under the sensors
	else if(sensors.val[max_i - 1] > sensors.val[max_i + 1]) // the line is on the left of the max sensor
	{
		sensor_relativ_pos = -sensors.val[max_i];
	}
	else // the line is on the right of the max sensor
	{
		sensor_relativ_pos = sensors.val[max_i];
	}

	pos = (max_i - 2) * MAX_FLOOR_SENSORS_VAL + sensor_relativ_pos;
	return pos;
}

void setFloorSensorsCalib(FloorSensorsCalib calib)
{
	current_floor_calib = calib;
}
