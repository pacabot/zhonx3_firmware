/*
 * calibration.c
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */
#include "middleware/telemeters/calibration.h"
#include "application/statistiques/statistiques.h"
#include "middleware/controls/motionControl/mainControl.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/bluetooth/bluetooth.h"
int sensorInit(void)
{
	return CALIBRATION_E_SUCCESS;
}
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
void telemeter_calibration (void)
{
	int values[2][NUMBER_OF_CASE]={{0}};
	int values_for_statistique_right[NUMBER_OF_MEASURE_BY_STEP]={0};
	int values_for_statistique_left[NUMBER_OF_MEASURE_BY_STEP]={0};
	int old_mesure, length;


	ssd1306DrawString(0,0,"Place the robot in",&Font_5x8);
	ssd1306DrawString(10,0,"30 cms of front wall",&Font_5x8);
	ssd1306DrawString(20,0,"and press 'RIGHT'",&Font_5x8);
	while(expanderJoyFiltered()!=JOY_RIGHT);


	telemetersInit();
	bluetoothPrintf("step|front left|front right");
	old_mesure=telemeters.end_of_conversion;

	for(int i=0;i<NUMBER_OF_CASE;i++)
	{
		for (int y = 0; y < NUMBER_OF_MEASURE_BY_STEP; ++y)
		{
			while(old_mesure==telemeters.end_of_conversion);
			values_for_statistique_left[y]=telemeters.left_front.telemeter_value;
			values_for_statistique_right[y]=telemeters.right_front.telemeter_value;
			old_mesure=telemeters.end_of_conversion;
			bluetoothPrintf("%2d.%2d|%10d|%d",i,y,values_for_statistique_left[y],values_for_statistique_right[y]);
		}

		length=NUMBER_OF_MEASURE_BY_STEP;
		eliminateExtremeValues(&(values_for_statistique_left),&length);
		values[0][i]= cAverage(values_for_statistique_left,length);

		length=NUMBER_OF_MEASURE_BY_STEP;
		eliminateExtremeValues(&(values_for_statistique_right),&length);
		values[1][i]= cAverage(values_for_statistique_right,length);

		move(0,NUMBER_OF_MILLIMETER_PER_LOOP,LOWSPEED,0);
	}
	bluetoothPrintf("\n\n\nfilterd measures :\n");
	for (int i = 0; i < NUMBER_OF_CASE; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,values[0][i],values[1][i]);
	}
	telemetersStop();
	printCurve(values[0],NUMBER_OF_CASE);
}
