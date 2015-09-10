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

walls cell_state = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};

//int telemetersWithOutNoise//TODO this function
int wallSensorsCalibration (void)
{
	//	int value_to_retest_front_right [NUMBER_OF_CELL+1];
	//	int length_front_right;
	//	int value_to_retest_front_left [NUMBER_OF_CELL+1];
	//	int length_front_left;
	//	int value_to_retest_diag_right [NUMBER_OF_CELL+1];
	//	int length_diag_right;
	//	int value_to_retest_diag_left [NUMBER_OF_CELL+1];
	//	int length_diag_left;

	ssd1306ClearScreen();
	ssd1306DrawString(0,0,"Place the robot front",&Font_5x8);
	ssd1306DrawString(0,10,"of wall and press 'RIGHT'",&Font_5x8);
	ssd1306Refresh();
	while(expanderJoyFiltered()!=JOY_RIGHT);

	ssd1306ClearScreen();
	ssd1306Printf(0,0,&Font_5x8,"Calibrating front sensors");
	ssd1306Refresh();

	mainControlInit();
	telemetersInit();
	telemetersStart();
	control_params.wall_follow_state = FALSE;
	HAL_Delay(1000);
	for(int i = 0; i < NUMBER_OF_CELL; i++)
	{
		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
		ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
		ssd1306Refresh();

		telemeter_FL_profile[i]=telemeters.FL.avrg;
		telemeter_FR_profile[i]=telemeters.FR.avrg;


		move(0,-NUMBER_OF_MILLIMETER_BY_LOOP,50,0);
		while(speed_control.end_control != 1);
	}
	//	int position_zhonx=NUMBER_OF_CELL;
	//	int distance;
	//	length_front_left=0;
	//	length_front_right=0;
	//	do
	//	{
	//		while (length_front_right>0 || length_front_left>0)
	//		{
	/*			if(length_front_left>=0 && \
//					((position_zhonx-value_to_retest_front_left[length_front_left]) < (position_zhonx-value_to_retest_front_right[length_front_right])\
//					|| length_front_right<=0))
	 *///			{
	//				distance=(position_zhonx-value_to_retest_front_left[length_front_left])*NUMBER_OF_MILLIMETER_BY_LOOP;
	//				bluetoothPrintf("position Zhonx=%d,\tvaleur gauche a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_left[length_front_left], distance);
	//				if (distance!=0)
	//				{
	//					move(0,(float)distance,10,0);
	//					while(speed_control.end_control != 1);
	//					position_zhonx=value_to_retest_front_left[length_front_left];
	//				}
	//
	//				telemeter_left_front_voltage[value_to_retest_front_left[length_front_left]]=telemeters.left_front.average_value;
	//
	//				length_front_left--;
	//			}
	//			else if (length_front_right>=0)
	//			{
	//				distance=(position_zhonx-value_to_retest_front_right[length_front_right])*NUMBER_OF_MILLIMETER_BY_LOOP;
	//				bluetoothPrintf("position Zhonx=%d,\tvaleur droite a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_right[length_front_right],distance);
	//				if(distance!=0)
	//				{
	//					move(0,distance,10,0);
	//					while(speed_control.end_control != 1);
	//					position_zhonx=value_to_retest_front_right[length_front_right];
	//				}
	//				telemeter_right_front_voltage[value_to_retest_front_right[length_front_right]]=telemeters.right_front.average_value;
	//
	//				length_front_right--;
	//			}
	//		}
	//		bluetoothPrintf("\n\n\nfilterd measures :\n");
	//		for (int i = 0; i < NUMBER_OF_CELL; ++i)
	//		{
	//			bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
	//		}
	//		length_front_left=0;
	//		length_front_right=0;
	//		for(int i = 1; i<(NUMBER_OF_CELL-1); i++)
	//		{
	//			if (telemeter_left_front_voltage[i] <= telemeter_left_front_voltage[i+1])
	//			{
	//				value_to_retest_front_left[length_front_left] = i;
	//				length_front_left ++;
	//				if( i==(NUMBER_OF_CELL-2) )
	//				{
	//					value_to_retest_front_left[length_front_left] = i+1;
	//					length_front_left ++;
	//				}
	//			}
	//			if (telemeter_right_front_voltage[i] <= telemeter_right_front_voltage[i+1])
	//			{
	//				value_to_retest_front_right[length_front_right] = i;
	//				length_front_right ++;
	//				if( i==(NUMBER_OF_CELL-2) )
	//				{
	//					value_to_retest_front_right[length_front_right] = i+1;
	//					length_front_right ++;
	//				}
	//			}
	//			if (telemeter_left_front_voltage[i-1] <= telemeter_left_front_voltage[i])
	//			{
	//				if ( i==1 )
	//				{
	//					value_to_retest_front_left[length_front_left] = i-1;
	//					length_front_left ++;
	//				}
	//				value_to_retest_front_left[length_front_left] = i;
	//				length_front_left ++;
	//			}
	//			if (telemeter_right_front_voltage[i-1] <= telemeter_right_front_voltage[i])
	//			{
	//				if( i==1 )
	//				{
	//					value_to_retest_front_right[length_front_right] = i-1;
	//					length_front_right ++;
	//				}
	//				value_to_retest_front_right[length_front_right] = i;
	//				length_front_right ++;
	//			}
	//		}
	//		length_front_left --;
	//		length_front_right --;
	//		bluetoothPrintf("nombre de valeur a retester : a gauche : %d, a d)roite %d\n",length_front_left,length_front_right);
	//	}while (length_front_right>0 || length_front_left>0);

	bluetoothPrintf("\n\n\nfilterd measures :\n");
	for (int i = 0; i < NUMBER_OF_CELL; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_FL_profile[i],telemeter_FR_profile[i]);
	}

	//diag telemeters calibration
	ssd1306ClearScreen();
	ssd1306Printf(0,0,&Font_5x8,"calibrating diag telemeters");
	ssd1306ProgressBar(10,10,0);
	ssd1306ProgressBar(10,40,50);
	ssd1306Refresh();
	while(expanderJoyFiltered()!=JOY_RIGHT);
	for(int i=0;i<NUMBER_OF_CELL;i++)
	{
		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
		ssd1306ProgressBar(10,40,50+(i*50)/NUMBER_OF_CELL);
		ssd1306Refresh();
		move(0,-sqrtf(2*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),50,0);
		while(speed_control.end_control != 1);
		telemeter_DL_profile[i]=telemeters.DL.avrg;
		telemeter_DR_profile[i]=telemeters.DR.avrg;
	}
	bluetoothPrintf("\n\n\nfilterd diag measures :\n");
	for (int i = 0; i < NUMBER_OF_CELL; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_DL_profile[i],telemeter_DR_profile[i]);
	}
	telemetersStop();
	motorsSleepDriver(ON);
	return WALL_SENSORS_E_SUCCESS;
}

void setCellState()
{
	memset(&cell_state, 0, sizeof(walls));
	if (telemeters.FL.dist_mm < DISTANCE_FIRST_WALL_FRONT)
		cell_state.front = WALL_PRESENCE;
	if (telemeters.FL.dist_mm < DISTANCE_SEGOND_WALL_FRONT)
		cell_state.next_front = WALL_PRESENCE;
	if (telemeters.DL.dist_mm < DISTANCE_WALL_DIAG)
		cell_state.left = WALL_PRESENCE;
	if (telemeters.DR.dist_mm < DISTANCE_WALL_DIAG)
		cell_state.right = WALL_PRESENCE;
}

void testTelemeterDistance()
{
	telemetersInit();
	telemetersStart();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();

		ssd1306Printf(0, 0 ,&Font_5x8,"F L :%d",(int)(telemeters.FL.dist_mm*10));
		ssd1306Printf(0, 10,&Font_5x8,"F R :%d",(int)(telemeters.FR.dist_mm*10));
		ssd1306Printf(0, 20,&Font_5x8,"D L :%d",(int)(telemeters.DL.dist_mm*10));
		ssd1306Printf(0, 30,&Font_5x8,"D R :%d",(int)(telemeters.DR.dist_mm*10));

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

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		setCellState();;
		ssd1306ClearScreen();
		if (cell_state.front == WALL_PRESENCE)
		{
			ssd1306FillRect(0,49,54,5);
		}
		else
		{
			ssd1306DrawRect(0,49,54,5);
		}
		switch (cell_state.next_front)
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
		switch (cell_state.left)
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
		switch (cell_state.right)
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
		ssd1306Printf(55, 0 ,&Font_5x8,"F L :%d",(int)(telemeters.FL.dist_mm*10));
		ssd1306Printf(55, 10,&Font_5x8,"F R :%d",(int)(telemeters.FR.dist_mm*10));
		ssd1306Printf(55, 20,&Font_5x8,"D L :%d",(int)(telemeters.DL.dist_mm*10));
		ssd1306Printf(55, 30,&Font_5x8,"D R :%d",(int)(telemeters.DR.dist_mm*10));
		ssd1306Refresh();
	}
	telemetersStop();
}

void testPostSensors()
{
	telemetersInit();
	telemetersStart();
	walls wall_saw;

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
//plf		wall_saw=getCellState();
		ssd1306ClearScreen();

		if (fabs(telemeters.DL.speed_mms) > 500)
		{
		ssd1306FillRect(0,49,5,5);
		}
//		else
//		{
//		ssd1306DrawRect(0,49,5,5);
//		}

//		ssd1306FillRect(0,0,5,5);
//		ssd1306DrawRect(0,0,5,5);

		if (fabs(telemeters.DR.speed_mms) > 500)
		{
		ssd1306FillRect(49,49,5,5);
		}
//		else
//		{
//		ssd1306DrawRect(49,49,5,5);
//		}

//		ssd1306FillRect(49,49,5,5);
//
//		ssd1306DrawRect(49,49,5,5);
//
//		ssd1306FillRect(49,0,5,5);
//
//		ssd1306DrawRect(49,0,5,5);


		ssd1306PrintInt(55, 0 , "", (int32_t) telemeters.FL.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 10, "", (int32_t) telemeters.DL.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 20, "", (int32_t) telemeters.DR.speed_mms, &Font_5x8);
		ssd1306PrintInt(55, 30, "", (int32_t) telemeters.FR.speed_mms, &Font_5x8);
		ssd1306Refresh();
	}
	telemetersStop();
}

telemetersStruct * getDistance_ptr(void)
{
	return &telemeters;
}
