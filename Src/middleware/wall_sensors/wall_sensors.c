/*
 * wall_sensors.c
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Application declarations */
#include "application/statistiques/statistiques.h"

/* Peripheral declarations */
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/motors/motors.h"

/* Middleware declarations */
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "middleware/wall_sensors/wall_sensors.h"

void wallSensorInit(void)
{
	memset(&telemeters, 0, sizeof(telemetersStruct));
//	static int old_value_DR	= 0;
//	static int old_value_DL	= 0;
//	static int old_value_FR	= 0;
//	static int old_value_FL	= 0;
//
//	static int cell_DR		= NUMBER_OF_CELL - 1;
//	static int cell_DL		= NUMBER_OF_CELL - 1;
//	static int cell_FR		= NUMBER_OF_CELL - 1;
//	static int cell_FL		= NUMBER_OF_CELL - 1;
//
//	char sens_DR	= 1;
//	char sens_DL	= 1;
//	char sens_FR	= 1;
//	char sens_FL	= 1;
//	resetTrendDist(&telemeters);
}

//int telemetersWithOutNoise//TODO this function
int wallSensorsCalibration (void)
{
////	int value_to_retest_front_right [NUMBER_OF_CELL+1]; //todo uncomment
////	int length_front_right;
////	int value_to_retest_front_left [NUMBER_OF_CELL+1];
////	int length_front_left;
////	int value_to_retest_diag_right [NUMBER_OF_CELL+1];
////	int length_diag_right;
////	int value_to_retest_diag_left [NUMBER_OF_CELL+1];
////	int length_diag_left;
//
//	ssd1306ClearScreen();
//	ssd1306DrawString(0,0,"Place the robot front",&Font_5x8);
//	ssd1306DrawString(0,10,"of wall and press 'RIGHT'",&Font_5x8);
//	ssd1306Refresh();
//	while(expanderJoyFiltered()!=JOY_RIGHT);
//
//	ssd1306ClearScreen();
//	ssd1306Printf(0,0,&Font_5x8,"Calibrating front sensors");
//	ssd1306Refresh();
//
//	mainControlInit();
//	telemetersInit();
//	telemetersStart();
//	control_params.speed_state = TRUE;
//	control_params.wall_follow_state = FALSE;
//	control_params.position_state = TRUE;
//	HAL_Delay(1000);
//	for(int i = 0; i < NUMBER_OF_CELL; i++)
//	{
//		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//		ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
//		ssd1306Refresh();
//
////		telemeter_FL_profile[i]=telemeters.FL.avrg;   //todo uncomment PLF
////		telemeter_FR_profile[i]=telemeters.FR.avrg;
//
//		move(0,-NUMBER_OF_MILLIMETER_BY_LOOP,50,0);
//		while(speed_control.end_control != 1);
//	}
////	int position_zhonx=NUMBER_OF_CELL;
////	int distance;
////	length_front_left=0;
////	length_front_right=0;
////	do
////	{
////		while (length_front_right>0 || length_front_left>0)
////		{
///*			if(length_front_left>=0 && \
////					((position_zhonx-value_to_retest_front_left[length_front_left]) < (position_zhonx-value_to_retest_front_right[length_front_right])\
////					|| length_front_right<=0))
//*///			{
////				distance=(position_zhonx-value_to_retest_front_left[length_front_left])*NUMBER_OF_MILLIMETER_BY_LOOP;
////				bluetoothPrintf("position Zhonx=%d,\tvaleur gauche a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_left[length_front_left], distance);
////				if (distance!=0)
////				{
////					move(0,(float)distance,10,0);
////					while(speed_control.end_control != 1);
////					position_zhonx=value_to_retest_front_left[length_front_left];
////				}
////
////				telemeter_left_front_voltage[value_to_retest_front_left[length_front_left]]=telemeters.left_front.average_value;
////
////				length_front_left--;
////			}
////			else if (length_front_right>=0)
////			{
////				distance=(position_zhonx-value_to_retest_front_right[length_front_right])*NUMBER_OF_MILLIMETER_BY_LOOP;
////				bluetoothPrintf("position Zhonx=%d,\tvaleur droite a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_right[length_front_right],distance);
////				if(distance!=0)
////				{
////					move(0,distance,10,0);
////					while(speed_control.end_control != 1);
////					position_zhonx=value_to_retest_front_right[length_front_right];
////				}
////				telemeter_right_front_voltage[value_to_retest_front_right[length_front_right]]=telemeters.right_front.average_value;
////
////				length_front_right--;
////			}
////		}
////		bluetoothPrintf("\n\n\nfilterd measures :\n");
////		for (int i = 0; i < NUMBER_OF_CELL; ++i)
////		{
////			bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
////		}
////		length_front_left=0;
////		length_front_right=0;
////		for(int i = 1; i<(NUMBER_OF_CELL-1); i++)
////		{
////			if (telemeter_left_front_voltage[i] <= telemeter_left_front_voltage[i+1])
////			{
////				value_to_retest_front_left[length_front_left] = i;
////				length_front_left ++;
////				if( i==(NUMBER_OF_CELL-2) )
////				{
////					value_to_retest_front_left[length_front_left] = i+1;
////					length_front_left ++;
////				}
////			}
////			if (telemeter_right_front_voltage[i] <= telemeter_right_front_voltage[i+1])
////			{
////				value_to_retest_front_right[length_front_right] = i;
////				length_front_right ++;
////				if( i==(NUMBER_OF_CELL-2) )
////				{
////					value_to_retest_front_right[length_front_right] = i+1;
////					length_front_right ++;
////				}
////			}
////			if (telemeter_left_front_voltage[i-1] <= telemeter_left_front_voltage[i])
////			{
////				if ( i==1 )
////				{
////					value_to_retest_front_left[length_front_left] = i-1;
////					length_front_left ++;
////				}
////				value_to_retest_front_left[length_front_left] = i;
////				length_front_left ++;
////			}
////			if (telemeter_right_front_voltage[i-1] <= telemeter_right_front_voltage[i])
////			{
////				if( i==1 )
////				{
////					value_to_retest_front_right[length_front_right] = i-1;
////					length_front_right ++;
////				}
////				value_to_retest_front_right[length_front_right] = i;
////				length_front_right ++;
////			}
////		}
////		length_front_left --;
////		length_front_right --;
////		bluetoothPrintf("nombre de valeur a retester : a gauche : %d, a d)roite %d\n",length_front_left,length_front_right);
////	}while (length_front_right>0 || length_front_left>0);
//
//	bluetoothPrintf("\n\n\nfilterd measures :\n");
//	for (int i = 0; i < NUMBER_OF_CELL; ++i)
//	{
//		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_FL_profile[i],telemeter_FR_profile[i]);
//	}
//
//	//diag telemeters calibration
//	ssd1306ClearScreen();
//	ssd1306Printf(0,0,&Font_5x8,"calibrating diag telemeters");
//	ssd1306ProgressBar(10,10,0);
//	ssd1306ProgressBar(10,40,50);
//	ssd1306Refresh();
//	while(expanderJoyFiltered()!=JOY_RIGHT);
//	for(int i=0;i<NUMBER_OF_CELL;i++)
//	{
//		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//		ssd1306ProgressBar(10,40,50+(i*50)/NUMBER_OF_CELL);
//		ssd1306Refresh();
//		move(0,-sqrtf(2*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),50,0);
//		while(speed_control.end_control != 1);
////		telemeter_DL_voltage[i]=telemeters.DL.avrg;	//todo uncomment PLF
////		telemeter_DR_voltage[i]=telemeters.DR.avrg;
//	}
//	bluetoothPrintf("\n\n\nfilterd diag measures :\n");
//	for (int i = 0; i < NUMBER_OF_CELL; ++i)
//	{
//		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_DL_voltage[i],telemeter_DR_voltage[i]);
//	}
//	telemetersStop();
//	motorsSleepDriver(ON);
	return WALL_SENSORS_E_SUCCESS;
}



//int getTelemetersDistance (telemetersStruct *telemeters)
//{
//	telemetersStop();
//	static int old_value_DR	= 0;
//	static int old_value_DL	= 0;
//	static int old_value_FR	= 0;
//	static int old_value_FL	= 0;
//
//	static int cell_DR		= NUMBER_OF_CELL - 1;
//	static int cell_DL		= NUMBER_OF_CELL - 1;
//	static int cell_FR		= NUMBER_OF_CELL - 1;
//	static int cell_FL		= NUMBER_OF_CELL - 1;
//
//	char sens_DR	= 1;
//	char sens_DL	= 1;
//	char sens_FR	= 1;
//	char sens_FL	= 1;
//
//	float value_DR;
//	float value_DL;
//	float value_FR;
//	float value_FL;
//
//	value_FL = telemeters->FL.avrg;
//	value_FR = telemeters->FR.avrg;
//	value_DL = telemeters->DL.avrg;
//	value_DR = telemeters->DR.avrg;
//
//	if(value_FL > old_value_FL)
//	{
//		sens_FL=-1;
//	}
//	if(value_FR > old_value_FR)
//	{
//		sens_FR=-1;
//	}
//	if(value_DL > old_value_DL)
//	{
//		sens_DL=-1;
//	}
//	if(value_DR > old_value_DR)
//	{
//		sens_DR=-1;
//	}
//
//
//	while ((value_FL > telemeter_FL_voltage[cell_FL]) || (value_FL < telemeter_FL_voltage[cell_FL + 1]))
//	{
//		cell_FL += sens_FL;
//		if (cell_FL < 0)
//		{
//			cell_FL = 0;
//			break;
//		}
//		else if (cell_FL >= NUMBER_OF_CELL)
//		{
//			cell_FL = NUMBER_OF_CELL;
//			break;
//		}
//	}
//
//	while ((value_FR > telemeter_FR_voltage[cell_FR]) || (value_FR < telemeter_FR_voltage[cell_FR + 1]))
//	{
//		cell_FR += sens_FR;
//		if (cell_FR < 0)
//		{
//			cell_FR=0;
//			break;
//		}
//		else if (cell_FR >= NUMBER_OF_CELL)
//		{
//			cell_FR=NUMBER_OF_CELL;
//			break;
//		}
//	}	while ((value_DL > telemeter_DL_voltage[cell_DL]) || (value_DL < telemeter_DL_voltage[cell_DL + 1]))
//	{
//		cell_DL += sens_DL;
//		if (cell_DL < 0)
//		{
//			cell_DL = 0;
//			break;
//		}
//		else if (cell_DL >= NUMBER_OF_CELL)
//		{
//			cell_DL = NUMBER_OF_CELL;
//			break;
//		}
//	}
//
//	while ((value_DR > telemeter_DR_voltage[cell_DR]) || (value_DR < telemeter_DR_voltage[cell_DR + 1]))
//	{
//		cell_DR += sens_DR;
//		if (cell_DR < 0)
//		{
//			cell_DR=0;
//			break;
//		}
//		else if (cell_DR >= NUMBER_OF_CELL)
//		{
//			cell_DR = NUMBER_OF_CELL;
//			break;
//		}
//	}
//
///*
// * 		(ya xb - xa yb - yc xb + yc xa)
// * xc= _________________________________
// * 				  (-yb + ya)
// * xc <- distance in millimeters
// * yc <- voltage measured
// *
// * xa <- distance in millimeters measured in the calibrate function
// * ya <- voltage measure in the calibrate function, the voltage correspond to the distance of xa
// *
// * xb <- distance in millimeters measured in the calibrate function, it's the distance xa+DISTANCE_BY_LOOP
// * yb <- voltage measure in the calibrate function, the voltage correspond to the distance of xb
// *
// * XXXX is the sensor reference : front_left , front_right , diag_right , diag_left
// *
// *
// * 		telemeter_XXXX_voltage[cell_XXXX] (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP telemeter_XXXX_voltage[cell_XXXX + 1] - value_XXX (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_XXXX cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP
// * xc= _______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
// * 																		-telemeter_XXX_voltage[cell_XXXX + 1] + telemeter_XXX_voltage[cell_XXXX]
// *
// */
//	telemeters->FL.dist_mm =
//			(telemeter_FL_profile[cell_FL] * (cell_FL + 1) * NUMBER_OF_MILLIMETER_BY_LOOP -
//					cell_FL * NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_FL_profile[cell_FL + 1] -
//					value_FL * (cell_FL + 1) * NUMBER_OF_MILLIMETER_BY_LOOP + (float)value_FL * cell_FL *
//					NUMBER_OF_MILLIMETER_BY_LOOP) /
//			(- telemeter_FL_profile[cell_FL + 1] + telemeter_FL_profile[cell_FL]);
//	old_value_FL = value_FL;
//
//	telemeters->FR.dist_mm =
//			(telemeter_FR_profile[cell_FR] * (cell_FR + 1) * NUMBER_OF_MILLIMETER_BY_LOOP -
//			cell_FR * NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_FR_profile[cell_FR + 1] -
//			value_FR * (cell_FR + 1) * NUMBER_OF_MILLIMETER_BY_LOOP + (float)value_FR * cell_FR *
//			NUMBER_OF_MILLIMETER_BY_LOOP) /
//			(-telemeter_FR_voltage[cell_FR + 1] + telemeter_FR_voltage[cell_FR]);
//	old_value_FR = value_FR;
//
//	telemeters->DL.dist_mm =
//			(telemeter_DL_voltage[cell_DL] * (cell_DL + 1) * NUMBER_OF_MILLIMETER_BY_LOOP -
//			cell_DL * NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_DL_voltage[cell_DL + 1] -
//			value_DL * (cell_DL + 1) * NUMBER_OF_MILLIMETER_BY_LOOP + (float)value_DL * cell_DL *
//			NUMBER_OF_MILLIMETER_BY_LOOP) /
//			(-telemeter_DL_voltage[cell_DL + 1] + telemeter_DL_voltage[cell_DL]);
//	old_value_DL = value_DL;
//
//	telemeters->DR.dist_mm =
//			(telemeter_DR_voltage[cell_DR] * (cell_DR + 1) * NUMBER_OF_MILLIMETER_BY_LOOP -
//			cell_DR * NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_DR_voltage[cell_DR + 1] -
//			value_DR * (cell_DR + 1)*NUMBER_OF_MILLIMETER_BY_LOOP + (float)value_DR * cell_DR *
//			NUMBER_OF_MILLIMETER_BY_LOOP) /
//			(-telemeter_DR_voltage[cell_DR + 1] + telemeter_DR_voltage[cell_DR]);
//	old_value_DR = value_DR;
//	telemetersStart();
//
//	return WALL_SENSORS_E_SUCCESS;
//}

walls getCellState()
{
//	telemetersStruct *ptr_distances = getDistance_ptr();
//	walls walls_position = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};
//
//	getTelemetersDistance(ptr_distances);
//	if (ptr_distances->FL.dist_mm < DISTANCE_FIRST_WALL_FRONT)
//	{
//		walls_position.front = WALL_PRESENCE;
//	}
//	if (ptr_distances->FL.dist_mm < DISTANCE_SEGOND_WALL_FRONT)
//		walls_position.next_front = WALL_PRESENCE;
//	if (ptr_distances->DL.dist_mm < DISTANCE_WALL_DIAG)
//		walls_position.left = WALL_PRESENCE;
//	if (ptr_distances->DR.dist_mm < DISTANCE_WALL_DIAG)
//		walls_position.right = WALL_PRESENCE;
//	return walls_position;
	}

void testTelemeterDistance()
{
	telemetersStruct *ptr_distances = getDistance_ptr();
	telemetersInit();
	telemetersStart();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
//		getTelemetersDistance(ptr_distances);  todo uncomment
//		ssd1306Printf(0,0,&Font_5x8,"D f l=%d",(int)(ptr_distances->FL.dist_mm)*10);   //todo uncomment PLF
//		ssd1306Printf(0,10,&Font_5x8,"D f r=%d",(int)(ptr_distances->FR.dist_mm)*10);
//		ssd1306Printf(0,20,&Font_5x8,"D d l=%d",(int)(ptr_distances->DL.dist_mm)*10);
//		ssd1306Printf(0,30,&Font_5x8,"D d r=%d",(int)(ptr_distances->DR.dist_mm)*10);

		ssd1306Printf(60,0,&Font_5x8,"V f l=%d",(int)(telemeters.FL.avrg));
		ssd1306Printf(60,10,&Font_5x8,"V f r=%d",(int)(telemeters.FR.avrg));
		ssd1306Printf(60,20,&Font_5x8,"V d l=%d",(int)(telemeters.DL.avrg));
		ssd1306Printf(60,30,&Font_5x8,"V d r=%d",(int)(telemeters.DR.avrg));


		ssd1306Refresh();
	}
	telemetersStop();
}

void testWallsSensors()
{
	telemetersInit();
	telemetersStart();
	wallSensorInit();
	walls wall_saw;
	telemetersStruct *ptr_distances = getDistance_ptr();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		//getTelemetersDistance(ptr_distances);   todo uncomment
		wall_saw=getCellState();
		ssd1306ClearScreen();
		if (wall_saw.front == WALL_PRESENCE)
		{
			ssd1306FillRect(0,49,54,5);
		}
		else
		{
			ssd1306DrawRect(0,49,54,5);
		}
		switch (wall_saw.next_front)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(0,0,54,5);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(0,0,54,5);
				break;
			default:
				break;
		}
		switch (wall_saw.left)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(0,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(0,0,5,54);
				break;
			default:
				break;
		}
		switch (wall_saw.right)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(49,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(49,0,5,54);
				break;
			default:
				break;
		}
//		ssd1306Printf(55, 0 ,&Font_5x8,"F L :%d",(int)(ptr_distances->FL.dist_mm*10));  //todo uncomment PLF
//		ssd1306Printf(55, 10,&Font_5x8,"F R :%d",(int)(ptr_distances->FR.dist_mm*10));
//		ssd1306Printf(55, 20,&Font_5x8,"D L :%d",(int)(ptr_distances->DL.dist_mm*10));
//		ssd1306Printf(55, 30,&Font_5x8,"D R :%d",(int)(ptr_distances->DR.dist_mm*10));
		ssd1306Refresh();
	}
	telemetersStop();
}

telemetersStruct * getDistance_ptr(void)
{
	return &telemeters;
}
