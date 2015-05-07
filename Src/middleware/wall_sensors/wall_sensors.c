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

int telemeter_right_front_voltage[NUMBER_OF_CELL]={3795,3786,3775,3773,3770,3769,3752,3736,3720,3710,3702,3682,3671,3643,3624,3605,3583,3551,3535,3512,3465,3459,3422,3400,3369,3321,3312,3253,3232,3183,3174,3090,3028,2945,2844,2769,2666,2586,2478,2398,2312,2234,2169,2095,2031,1959,1897,1836,1785,1733,1676,1640,1595,1550,1511,1469,1444,1407,1368,1345,1307,1286,1251,1230,1205,1178,1156,1137,1114,1091,1074,1039,1025,994,967,961,930,916,903,883,863,848,823,809,789,771,747,733,709,698,689,675,658,641,633,620,607,600,500,488};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={3780,3772,3763,3751,3751,3750,3734,3718,3716,3708,3690,3676,3657,3640,3616,3589,3572,3550,3536,3504,3475,3436,3408,3376,3347,3311,3255,3170,3062,2953,2843,2745,2645,2564,2472,2374,2299,2213,2134,2069,1980,1915,1844,1784,1723,1670,1614,1565,1521,1466,1417,1379,1345,1289,1255,1214,1174,1147,1116,1096,1067,1034,1022,999,975,959,942,915,900,882,865,838,825,807,787,770,758,726,720,700,693,666,657,640,618,614,591,587,563,559,536,524,515,509,493,486,482,473,461,451};
int telemeter_right_diag_voltage[NUMBER_OF_CELL];
int telemeter_left_diag_voltage[NUMBER_OF_CELL];

int wallSensorsCalibration (void)
{
	int value_to_retest_front_right [NUMBER_OF_CELL+1];
	int length_front_right;
	int value_to_retest_front_left [NUMBER_OF_CELL+1];
	int length_front_left;
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

	telemetersInit();
	mainControlInit();
	HAL_Delay(1000);
	for(int i = 0; i < NUMBER_OF_CELL; i++)
	{
		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
		ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
		ssd1306Refresh();

		telemeter_left_front_voltage[i]=telemeters.left_front.average_value;
		telemeter_right_front_voltage[i]=telemeters.right_front.average_value;

		move(0,-NUMBER_OF_MILLIMETER_BY_LOOP,50,0);
		while(speed_control.end_control != 1);
	}
	int position_zhonx=NUMBER_OF_CELL;
	int distance;
	length_front_left=0;
	length_front_right=0;
//	do
//	{
//		while (length_front_right>0 || length_front_left>0)
//		{
//			if(length_front_left>=0 && \
//					((position_zhonx-value_to_retest_front_left[length_front_left]) < (position_zhonx-value_to_retest_front_right[length_front_right])\
//					|| length_front_right<=0))
//			{
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
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
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
		move(0,-sqrtf(2*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),10,0);
		telemeter_left_diag_voltage[i]=telemeters.left_diag.average_value;
		telemeter_right_diag_voltage[i]=telemeters.right_diag.average_value;
	}
	bluetoothPrintf("\n\n\nfilterd diag measures :\n");
	for (int i = 0; i < NUMBER_OF_CELL; ++i)
	{
		bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_diag_voltage[i],telemeter_right_diag_voltage[i]);
	}
	telemetersStop();
	motorsSleepDriver(ON);
	return WALL_SENSORS_E_SUCCESS;
}


/*
 * formule
 * y=ax+b
 *
 * 	  yb-ya
 * a=_______
 *    xb-xa
 *
 * b=y-ax
 *
 * 		  yb-ya
 * b=ya- ________
 * 		  xb-xa
 */

int getTelemetersDistance (telemetersDistancesTypeDef *telemeters_distances)
{
	static int old_voltage_diag_right	= 0;
	static int old_voltage_diag_left	= 0;
	static int old_voltage_front_right	= 0;
	static int old_voltage_front_left	= 0;

	static int cell_diag_right		= NUMBER_OF_CELL - 1;
	static int cell_diag_left		= NUMBER_OF_CELL - 1;
	static int cell_front_right		= NUMBER_OF_CELL - 1;
	static int cell_front_left		= NUMBER_OF_CELL - 1;

	char sens_diag_right	= 1;
	char sens_diag_left		= 1;
	char sens_front_right	= 1;
	char sens_front_left	= 1;

	int value_diag_right;
	int value_diag_left;
	int value_front_right;
	int value_front_left;

	value_front_left = telemeters.left_front.average_value;
	value_front_right = telemeters.right_front.average_value;
	value_diag_left = telemeters.left_diag.average_value;
	value_diag_right = telemeters.right_diag.average_value;
	bluetoothPrintf("%d,\t%d",value_front_left, value_front_right);
	if(value_front_left > old_voltage_front_left)
	{
		sens_front_left=-1;
	}
	if(value_front_right > old_voltage_front_right)
	{
		sens_front_right=-1;
	}
	if(value_diag_left > old_voltage_diag_left)
	{
		sens_diag_left=-1;
	}
	if(value_diag_right > old_voltage_diag_right)
	{
		sens_diag_right=-1;
	}
	while ((value_front_left>telemeter_left_front_voltage[cell_front_left]) || (value_front_left<telemeter_left_front_voltage[cell_front_left + 1]) )
	{
		cell_front_left += sens_front_left;
		if (cell_front_left < 0)
		{
			cell_front_left=0;
			break;
		}
		else if (cell_front_left >= NUMBER_OF_CELL)
		{
			cell_front_left=NUMBER_OF_CELL;
			break;
		}
	}

	while ((value_front_right>telemeter_right_front_voltage[cell_front_right]) || (value_front_right<telemeter_right_front_voltage[cell_front_right + 1]) )
	{
		cell_front_right += sens_front_right;
		if (cell_front_right < 0)
		{
			cell_front_right=0;
			break;
		}
		else if (cell_front_right >= NUMBER_OF_CELL)
		{
			cell_front_right=NUMBER_OF_CELL;
			break;
		}
	}	while ((value_diag_left>telemeter_left_diag_voltage[cell_diag_left]) || (value_diag_left<telemeter_left_diag_voltage[cell_diag_left + 1]) )
	{
		cell_diag_left += sens_diag_left;
		if (cell_diag_left < 0)
		{
			cell_diag_left=0;
			break;
		}
		else if (cell_diag_left >= NUMBER_OF_CELL)
		{
			cell_diag_left=NUMBER_OF_CELL;
			break;
		}
	}

	while ((value_diag_right>telemeter_right_diag_voltage[cell_diag_right]) || (value_diag_right<telemeter_right_diag_voltage[cell_diag_right + 1]) )
	{
		cell_diag_right += sens_diag_right;
		if (cell_diag_right < 0)
		{
			cell_diag_right=0;
			break;
		}
		else if (cell_diag_right >= NUMBER_OF_CELL)
		{
			cell_diag_right=NUMBER_OF_CELL;
			break;
		}
	}

/*
 * 		(ya xb - xa yb - yc xb + yc xa)
 * xc= _________________________________
 * 				  (-yb + ya)
 * xc <- distance in millimeters
 * yc <- voltage measured
 *
 * xa <- distance in millimeters measured in the calibrate function
 * ya <- voltage measure in the calibrate function, the voltage correspond to the distance of xa
 *
 * xb <- distance in millimeters measured in the calibrate function, it's the distance xa+DISTANCE_BY_LOOP
 * yb <- voltage measure in the calibrate function, the voltage correspond to the distance of xb
 *
 * XXX is the capteur reference : front_left , front_right , diag_right , diag_left
 *
 *
 * 		telemeter_XXX_voltage[cell_XXX] (cell_XXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_XXX*NUMBER_OF_MILLIMETER_BY_LOOP telemeter_XXX_voltage[cell_XXX + 1] - value_XXX (cell_XXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_XXX cell_XXX*NUMBER_OF_MILLIMETER_BY_LOOP
 * xc= ________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
 * 																		-telemeter_XXX_voltage[cell_XXX + 1] + telemeter_XXX_voltage[cell_XXX]
 *
 */

	telemeters_distances->distance_front_left = (telemeter_left_front_voltage[cell_front_left] * (cell_front_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_front_left*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_left_front_voltage[cell_front_left + 1] - value_front_left * (cell_front_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_front_left * cell_front_left*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_left_front_voltage[cell_front_left + 1] + telemeter_left_front_voltage[cell_front_left]);
	old_voltage_front_left = value_front_left;

	telemeters_distances->distance_front_right = (telemeter_right_front_voltage[cell_front_right] * (cell_front_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_front_right*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_right_front_voltage[cell_front_right + 1] - value_front_right * (cell_front_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_front_right * cell_front_right*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_right_front_voltage[cell_front_right + 1] + telemeter_right_front_voltage[cell_front_right]);
	old_voltage_front_right = value_front_right;

	telemeters_distances->distance_diag_left = (telemeter_left_diag_voltage[cell_diag_left] * (cell_diag_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_diag_left*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_left_diag_voltage[cell_diag_left + 1] - value_diag_left * (cell_diag_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_diag_left * cell_diag_left*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_left_diag_voltage[cell_diag_left + 1] + telemeter_left_diag_voltage[cell_diag_left]);
	old_voltage_diag_left = value_diag_left;

	telemeters_distances->distance_diag_right = (telemeter_right_diag_voltage[cell_diag_right] * (cell_diag_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_diag_right*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_right_diag_voltage[cell_diag_right + 1] - value_diag_right * (cell_diag_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_diag_right * cell_diag_right*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_right_diag_voltage[cell_diag_right + 1] + telemeter_right_diag_voltage[cell_diag_right]);
	old_voltage_diag_right = value_diag_right;
	return WALL_SENSORS_E_SUCCESS;
}

walls getWallsPosition()
{
	telemetersDistancesTypeDef telemeters_distances;
	walls walls_position = {NO_WALL,NO_WALL,NO_WALL};

	getTelemetersDistance(&telemeters_distances);
	if (telemeters_distances.distance_front_left < DISTANCE_WALL_FRONT)
		walls_position.front = WALL_KNOW;
	if (telemeters_distances.distance_diag_left < DISTANCE_WALL_DIAG)
		walls_position.left = WALL_KNOW;
	if (telemeters_distances.distance_front_right < DISTANCE_WALL_DIAG)
		walls_position.right = WALL_KNOW;
	return walls_position;
}

void testTelemeterDistance()
{
	telemetersDistancesTypeDef telemeters_distances;
	telemetersInit();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		getTelemetersDistance(&telemeters_distances);
		ssd1306Printf(0,0,&Font_5x8,"distancd left = %d",(int)(telemeters_distances.distance_front_left));
		ssd1306Printf(0,10,&Font_5x8,"distancd right = %d",(int)(telemeters_distances.distance_front_right));
		ssd1306Refresh();
		//bluetoothPrintf("distance left :%d\tright : %d\n",(int)telemeters_distances.distance_front_left,(int)telemeters_distances.distance_front_right);
	}
	telemetersStop();
}

void testWallsSensors()
{
	telemetersInit();
	walls wall_see;
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		wall_see=getWallsPosition();
		ssd1306ClearScreen();
		if(wall_see.front == WALL_KNOW)
			ssd1306FillRect(0,0,54,5);
		else
			ssd1306DrawRect(0,0,54,5);

		if(wall_see.left == WALL_KNOW)
			ssd1306FillRect(44,0,5,54);
		else
			ssd1306DrawRect(44,0,5,54);

		if(wall_see.right == WALL_KNOW)
			ssd1306FillRect(0,0,5,54);
		else
			ssd1306DrawRect(0,0,5,54);
		ssd1306Refresh();
	}
	telemetersStop();
}
