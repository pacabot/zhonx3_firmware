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
#include "stm32f4xx_hal.h"
#include "peripherals/motors/motors.h"

int telemeter_right_front_voltage[DISTANCE_MEASURED]={3128,2774,1389,719,403,266,137,99,45,34};
int telemeter_left_front_voltage[DISTANCE_MEASURED]={2883,2856,1917,1092,679,439,264,213,131,101};
int telemeter_right_diag_voltage[DISTANCE_MEASURED];
int telemeter_left_diag_voltage[DISTANCE_MEASURED];

int telemeterCalibration (void)
{
	int value_to_retest_front_right [NUMBER_OF_CASE+1];
	int length_front_right;
	int value_to_retest_front_left [NUMBER_OF_CASE+1];
	int length_front_left;
//	int value_to_retest_diag_right [NUMBER_OF_CASE+1];
//	int length_diag_right;
//	int value_to_retest_diag_left [NUMBER_OF_CASE+1];
//	int length_diag_left;

	ssd1306ClearScreen();
	ssd1306DrawString(0,0,"Place the robot in",&Font_5x8);
	ssd1306DrawString(0,10,"30 cms of front wall",&Font_5x8);
	ssd1306DrawString(0,20,"and press 'RIGHT'",&Font_5x8);
	ssd1306Refresh();
	while(expanderJoyFiltered()!=JOY_RIGHT);
#if DEBUG_WALL_SENSOR>1
	bluetoothPrintf("step|front left|front right");
#endif
	telemetersInit();
	mainControlInit();
	HAL_Delay(500);
	for(int i = 0; i < NUMBER_OF_CASE; i++)
	{
		getTelemeterValueWithoutAmbientLight(&telemeter_left_front_voltage[i], &telemeter_right_front_voltage[i], &telemeter_left_diag_voltage[i], &telemeter_right_diag_voltage[i],NUMBER_OF_MEASURE_BY_STEP);

		move(0,-NUMBER_OF_MILLIMETER_PER_LOOP,50,0);
		while(speed_control.end_control != 1);
	}
	motorsSleepDriver(ON);
	int position_zhonx=NUMBER_OF_CASE;
	length_front_left=0;
	length_front_right=0;
	do
	{
		while (length_front_right>0 || length_front_left>0)
		{
			if(length_front_left>=0 && \
					((position_zhonx-value_to_retest_front_left[length_front_left]) < (position_zhonx-value_to_retest_front_right[length_front_right])\
					|| length_front_right<=0))
			{
				bluetoothPrintf("position Zhonx=%d,\tvaleur gauche a re tester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_left[length_front_left], (position_zhonx-value_to_retest_front_left[length_front_left])*NUMBER_OF_MILLIMETER_PER_LOOP);
				motorsSleepDriver(OFF);
				move(0,(position_zhonx-value_to_retest_front_left[length_front_left])*NUMBER_OF_MILLIMETER_PER_LOOP,10,0);
				while(speed_control.end_control != 1);
				motorsSleepDriver(ON);
				position_zhonx=value_to_retest_front_left[length_front_left];

				int value_unuse;
				getTelemeterValueWithoutAmbientLight(&telemeter_left_front_voltage[value_to_retest_front_left[length_front_left]],&value_unuse,&value_unuse,&value_unuse,NUMBER_OF_MEASURE_BY_STEP);

				length_front_left--;
			}
			else if (length_front_right>=0)
			{
				bluetoothPrintf("position Zhonx=%d,\tvaleur droite a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_right[length_front_right], (position_zhonx-value_to_retest_front_right[length_front_right])*NUMBER_OF_MILLIMETER_PER_LOOP);
				motorsSleepDriver(OFF);
				move(0,(position_zhonx-value_to_retest_front_right[length_front_right])*NUMBER_OF_MILLIMETER_PER_LOOP,10,0);
				while(speed_control.end_control != 1);
				motorsSleepDriver(ON);
				position_zhonx=value_to_retest_front_right[length_front_right];

				int value_unuse;
				getTelemeterValueWithoutAmbientLight(&value_unuse,&telemeter_right_front_voltage[value_to_retest_front_right[length_front_right]],&value_unuse,&value_unuse,NUMBER_OF_MEASURE_BY_STEP);

				length_front_right--;
			}
		}
		#if DEBUG_WALL_SENSOR>1
			bluetoothPrintf("\n\n\nfilterd measures :\n");
			for (int i = 0; i < NUMBER_OF_CASE; ++i)
			{
				bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
			}
		#endif
		length_front_left=0;
		length_front_right=0;
		for(int i = 1; i<(NUMBER_OF_CASE-1); i++)
		{
			if (telemeter_left_front_voltage[i] <= telemeter_left_front_voltage[i+1])
			{
				if( i==(NUMBER_OF_CASE-2) )
				{
					value_to_retest_front_left[length_front_left] = i+1;
					length_front_left ++;
				}
				value_to_retest_front_left[length_front_left] = i;
				length_front_left ++;
			}
			if (telemeter_right_front_voltage[i] <= telemeter_right_front_voltage[i+1])
			{

				if( i==(NUMBER_OF_CASE-2) )
				{
					value_to_retest_front_right[length_front_right] = i+1;
					length_front_right ++;
				}
				value_to_retest_front_right[length_front_right] = i;
				length_front_right ++;
			}
			if (telemeter_left_front_voltage[i-1] <= telemeter_left_front_voltage[i])
			{
				if ( i==1 )
				{
					value_to_retest_front_left[length_front_left] = i-1;
					length_front_left ++;
				}
				value_to_retest_front_left[length_front_left] = i;
				length_front_left ++;
			}
			if (telemeter_right_front_voltage[i-1] <= telemeter_right_front_voltage[i])
			{
				if( i==1 )
				{
					value_to_retest_front_right[length_front_right] = i-1;
					length_front_right ++;
				}
				value_to_retest_front_right[length_front_right] = i;
				length_front_right ++;
			}
		}
		length_front_left --;
		length_front_right --;
		bluetoothPrintf("nombre de valeur a re tester : a gauche : %d, a droite %d\n",length_front_left,length_front_right);
	}while (length_front_right>0 || length_front_left>0);
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
	return WALL_SENSORS_E_SUCCESS;
}
int getTelemetersDistance (float *distance_front_left, float *distance_front_right, float *distance_diag_left, float *distance_diag_right, int *precision)
{
	static int old_voltage_diag_right	= 0;
	static int old_voltage_diag_left	= 0;
	static int old_voltage_front_right	= 0;
	static int old_voltage_front_left	= 0;

	static int old_case_diag_right		= NUMBER_OF_CASE;
	static int old_case_diag_left		= NUMBER_OF_CASE;
	static int old_case_front_right		= NUMBER_OF_CASE;
	static int old_case_front_left		= NUMBER_OF_CASE;

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

	if(value_front_left>old_voltage_front_left)
		sens_front_left=-1;
	if(value_front_right>old_voltage_front_right)
		sens_front_right=-1;
	if(value_diag_left>old_voltage_diag_left)
		sens_diag_left=-1;
	if(value_diag_right>old_voltage_diag_right)
		sens_diag_right=-1;
	while ((value_front_left<=telemeter_left_front_voltage[old_case_front_left+1]) || (value_front_left>telemeter_left_front_voltage[old_case_front_left]))
		{
			old_case_front_left += sens_front_left;
			if (old_case_front_left < 0)
			{
				old_case_front_left=0;
				break;
			}
			else if (old_case_front_left > NUMBER_OF_CASE)
			{
				old_case_front_left=NUMBER_OF_CASE;
				break;
			}
		}
	*distance_front_left=old_case_front_left;
	old_voltage_front_left = value_front_left;
	return WALL_SENSORS_E_SUCCESS;
}
void testTelemeterDistance()
{
	telemetersInit();
	float unused;
	int unusedi;
	float distance;
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		getTelemetersDistance(&distance,&unused,&unused,&unused,&unusedi);
		ssd1306Printf(0,0,&Font_5x8,"distancd = %d",distance);
		ssd1306Refresh();
	}
	telemetersStop();
}
