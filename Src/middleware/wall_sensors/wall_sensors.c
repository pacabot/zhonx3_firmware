/*
 * wall_sensors.c
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */
#include "middleware/wall_sensors/wall_sensors.h"
#include "application/statistiques/statistiques.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "config/basetypes.h"

int telemeter_right_front_voltage[DISTANCE_MEASURED];
int telemeter_left_front_voltage[DISTANCE_MEASURED];
int telemeter_right_diag_voltage[DISTANCE_MEASURED];
int telemeter_left_diag_voltage[DISTANCE_MEASURED];

int telemeterCalibration (void)
{
	telemetersInit();
	mainControlInit();
	ssd1306ClearScreen();
	ssd1306DrawString(0,0,"Place the robot in",&Font_5x8);
	ssd1306DrawString(0,10,"30 cms of front wall",&Font_5x8);
	ssd1306DrawString(0,20,"and press 'RIGHT'",&Font_5x8);
	ssd1306Refresh();
	while(expanderJoyFiltered()!=JOY_RIGHT);
#if DEBUG_WALL_SENSOR>1
	bluetoothPrintf("step|front left|front right");
#endif
#if DEBUG_WALL_SENSOR>2
		bluetoothPrintf("step by step measures");
#endif
		HAL_Delay(500);
	for(int i = NUMBER_OF_CASE; i > 0; i--)
	{
		getTelemeterValueWithoutAmbientLight(&telemeter_left_front_voltage[i], &telemeter_right_front_voltage[i], &telemeter_left_diag_voltage[i], &telemeter_right_diag_voltage[i],5000);

		move(0,NUMBER_OF_MILLIMETER_PER_LOOP,50,0);
		while(speed_control.end_control != 1);
	}
#if DEBUG_WALL_SENSOR>1
	bluetoothPrintf("\n\n\nfilterd measures :\n");
	for (int i = 0; i < NUMBER_OF_CASE; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
	}
#endif
	telemetersStop();
	motorsSleepDriver(ON);
	return WALL_SENSORS_E_SUCCESS;
}

int getTelemetersDistance (float *distance_front_left, float *distance_front_right, float *distance_diag_left, float *distance_diag_right, int *precision)
{
	static int old_value_diag_right		= 0;
	static int old_value_diag_left		= 0;
	static int old_value_front_right	= 0;
	static int old_value_front_left		= 0;
	void printCurve (int *listOfPoint,int length)
	{
		int i;
		ssd1306ClearScreen();
		for ( i = 0; i <= length; i++)
		{
			ssd1306DrawPixel(i,listOfPoint[i]);
		}
		ssd1306Refresh();
	}
	static int old_case_diag_right		= 0;
	static int old_case_diag_left		= 0;
	static int old_case_front_right		= 0;
	static int old_case_front_left		= 0;

	static char sens = 0;


	char sens_diag_right	= 1;
	char sens_diag_left		= 1;
	char sens_front_right	= 1;
	char sens_front_left	= 1;

	int value_diag_right;
	int value_diag_left;
	int value_front_right;
	int value_front_left;

	int case_diag_right;
	int case_diag_left;
	int case_front_right;
	int case_front_left;
	int rv;

	rv=getTelemeterValueWithoutAmbientLight(&value_front_left, &value_front_right, &value_diag_left, &value_diag_right, 10);
	if (rv != WALL_SENSORS_E_SUCCESS)
		return rv;
	if(sens == 0)
	{
		if(telemeter_left_front_voltage[0] > telemeter_left_front_voltage[1])
			sens=-1;
		else
			sens=1;
	}

	if(value_front_left>old_value_front_left)
		sens_front_left=-1;
	if(value_front_right>old_value_front_right)
		sens_front_right=-1;
	if(value_diag_left>old_value_diag_left)
		sens_diag_left=-1;
	if(value_diag_right>old_value_diag_right)
		sens_diag_right=-1;


	return WALL_SENSORS_E_SUCCESS;
}

int getTelemeterValueWithoutAmbientLight (int *value_front_left, int *value_front_right, int *value_diag_left, int *value_diag_right, int number_of_measure)
{
	int old_mesure, length;
	int values_for_statistique_front_left [number_of_measure], values_for_statistique_front_right [number_of_measure];
	int values_for_statistique_diag_left [number_of_measure], values_for_statistique_diag_right [number_of_measure];
	old_mesure=telemeters.end_of_conversion;
	for (int y = 0; y < number_of_measure; ++y)
	{
		while(old_mesure==telemeters.end_of_conversion);
		values_for_statistique_front_left[y]=telemeters.left_front.telemeter_value-telemeters.ref.telemeter_value;
		values_for_statistique_front_right[y]=telemeters.right_front.telemeter_value-telemeters.ref.telemeter_value;
		values_for_statistique_diag_left[y]=telemeters.left_diag.telemeter_value-telemeters.ref.telemeter_value;
		values_for_statistique_diag_right[y]=telemeters.right_diag.telemeter_value-telemeters.ref.telemeter_value;
		old_mesure=telemeters.end_of_conversion;
	#if DEBUG_WALL_SENSOR>2
		bluetoothPrintf("%d|%d|%d\n",y,values_for_statistique_front_left[y],values_for_statistique_front_right[y]);
	#endif
	#if DEBUG_WALL_SENSOR>1
		ssd1306ClearScreen();
		ssd1306Printf(0,0,&Font_5x8,"front left  : %d",values_for_statistique_front_left[y]);
		ssd1306Printf(0,0,&Font_5x8,"front right : %d",values_for_statistique_front_right[y]);
		ssd1306Printf(0,0,&Font_5x8,"diag left   : %d",values_for_statistique_diag_left[y]);
		ssd1306Printf(0,0,&Font_5x8,"diag right  : %d",values_for_statistique_diag_right[y]);
		ssd1306Refresh();
	#endif
	}

	length=number_of_measure;
	eliminateExtremeValues(values_for_statistique_front_left,&length);
	*value_front_left = cAverage(values_for_statistique_front_left,length);

	length=number_of_measure;
	eliminateExtremeValues(values_for_statistique_front_right,&length);
	*value_front_right = cAverage(values_for_statistique_front_right,length);

	length=number_of_measure;
	eliminateExtremeValues(values_for_statistique_diag_left,&length);
	*value_diag_left = cAverage(values_for_statistique_diag_left,length);

	length=number_of_measure;
	eliminateExtremeValues(values_for_statistique_diag_right,&length);
	*value_diag_right = cAverage(values_for_statistique_diag_right,length);
	bluetoothPrintf("|%d|%d\n",*value_front_left,value_front_right);
	return WALL_SENSORS_E_SUCCESS;
}
