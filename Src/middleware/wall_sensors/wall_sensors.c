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

#ifdef DARK
int telemeter_right_front_voltage[NUMBER_OF_CELL]={3834,3822,3551,3759,3724,3762,3748,3745,3752,3741,3696,3735,3717,3686,3670,3656,3630,3600,3554,3530,3479,3446,3385,3286,3157,2990,2810,2728,2611,2496,2389,2221,2198,2101,2038,1962,1887,1820,1756,1703,1640,1588,1536,1495,1395,1387,1354,1308,1242,1195,1198,1156,1117,1087,1050,1020,1001,951,936,901,876,846,818,813,778,746,733,712,691,681,666,645,624,616,605,592,573,566,557,546,537,529,521,514,511,496,489,480,471,465,449,450,436,433,420,407,403,401,390,379};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={3850,3842,3786,3777,3780,3782,3767,3767,3775,3765,3773,3758,3748,3736,3727,3722,3704,3705,3690,3665,3640,3648,3599,3585,3519,3507,3469,3365,3397,3328,3206,3060,3067,3105,2916,2766,2780,2634,2588,2453,2414,2285,2252,2138,1964,2039,1904,1885,1860,1616,1691,1631,1605,1541,1545,1502,1408,1412,1333,1310,1277,1261,1210,1133,1115,1125,1074,1041,1014,973,949,944,911,901,883,865,857,828,810,797,783,773,760,750,731,724,713,700,689,680,676,640,653,636,625,621,604,596,592,578};
int telemeter_right_diag_voltage[NUMBER_OF_CELL]={3814,3570,3843,3832,3578,3578,3821,3306,3804,3799,3796,3282,3773,3756,3751,3750,3464,3485,3720,3698,3691,3192,3415,3391,3571,3331,3304,3272,3398,3092,2936,2976,2825,2697,2372,2289,2145,2041,1984,1996,1805,1838,1763,1677,1536,1533,1409,1439,1297,1359,1226,1263,1144,1179,1145,1036,1009,1038,979,907,940,909,863,849,828,804,740,769,749,695,714,678,666,686,635,661,605,617,622,608,593,540,528,548,532,516,500,452,447,464,448,436,415,421,394,409,396,374,371,372};
int telemeter_left_diag_voltage[NUMBER_OF_CELL]={3863,3850,3865,3844,3843,3827,3826,3814,3813,3799,3805,3794,3779,3780,3764,3769,3762,3758,3748,3743,3735,3731,3727,3722,3714,3708,3711,3703,3693,3688,3675,3670,3653,3644,3630,3612,3587,3564,3547,3511,3490,3451,3420,3397,3325,3290,3180,3049,2961,2850,2868,2717,2629,2542,2474,2402,2309,2233,2179,2061,2009,1938,1859,1836,1788,1741,1686,1637,1574,1585,1481,1455,1423,1405,1351,1338,1314,1306,1286,1245,1162,1204,1177,1153,1121,1088,1069,1048,1025,1002,978,953,932,915,898,871,861,836,845,836};
#else
int telemeter_right_front_voltage[NUMBER_OF_CELL]={3803,3740,3756,3730,3674,3292,3446,3673,3609,3571,3582,3533,3504,3463,3413,3346,3287,3191,3099,2935,2762,2643,2501,2375,2250,2140,1989,1941,1848,1735,1679,1615,1534,1470,1418,1357,1296,1245,1194,1151,1099,1063,1033,998,973,926,898,867,842,804,787,768,741,725,705,668,667,655,616,604,594,571,561,545,524,519,496,479,464,452,441,435,435,415,413,387,384,367,358,369,341,345,325,330,312,313,312,315,300,288,283,286,272,276,262,275,261,265,245,243};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={3821,3794,3783,3775,3750,3748,3726,3737,3698,3699,3709,3682,3666,3642,3636,3592,3584,3504,3492,3397,3440,3352,3329,3288,3160,3054,2860,2724,2590,2420,2407,2270,2162,2127,1956,1915,1836,1766,1687,1623,1590,1539,1436,1416,1330,1349,1299,1261,1224,1185,1150,1094,1079,1032,1004,963,950,909,911,862,824,837,803,776,747,741,715,705,685,665,642,624,598,593,576,570,556,544,527,514,509,477,485,474,468,457,452,437,438,426,424,414,411,398,398,389,384,376,369,366};
int telemeter_right_diag_voltage[NUMBER_OF_CELL]={3876,3863,3602,3838,3571,3574,3827,3815,3804,3787,3788,3290,3532,3760,3675,3731,3607,3453,3434,3665,3449,3615,3432,3420,3494,3361,3442,3187,3338,3266,3159,3043,2870,2714,2410,2271,2250,2156,1985,1883,1625,1790,1759,1642,1508,1526,1380,1395,1340,1295,1287,1245,1208,1090,1057,1081,1035,1025,941,944,888,921,822,804,838,803,764,766,688,654,682,677,644,642,626,604,577,580,567,546,539,494,519,441,464,486,465,465,454,444,429,422,382,398,388,367,368,340,343,346};
int telemeter_left_diag_voltage[NUMBER_OF_CELL]={3868,3850,3861,3842,3821,3833,3835,3827,3817,3795,3799,3790,3783,3778,3768,3753,3744,3737,3725,3719,3718,3706,3699,3695,3678,3657,3634,3613,3590,3554,3536,3508,3471,3431,3388,3298,3219,3139,3007,2870,2744,2621,2509,2407,2307,2217,2128,2053,1989,1919,1842,1772,1711,1660,1635,1565,1518,1472,1423,1370,1337,1290,1243,1199,1158,1123,1094,1065,1034,1014,995,961,944,916,900,880,871,841,817,809,787,770,750,745,743,735,714,704,693,684,664,630,630,619,606,597,586,573,567,558};
#endif

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
//	int position_zhonx=NUMBER_OF_CELL;
//	int distance;
//	length_front_left=0;
//	length_front_right=0;
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
		move(0,-sqrtf(2*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),50,0);
		while(speed_control.end_control != 1);
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
	walls walls_position = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};

	getTelemetersDistance(&telemeters_distances);
	if (telemeters_distances.distance_front_left < DISTANCE_FIRST_WALL_FRONT)
	{
		walls_position.front = WALL_KNOW;

		walls_position.next_front = NO_KNOW;
		walls_position.left = NO_KNOW;
		walls_position.right = NO_KNOW;
	}
	else
	{
		if (telemeters_distances.distance_front_left < DISTANCE_SEGOND_WALL_FRONT)
			walls_position.next_front = WALL_KNOW;
		if (telemeters_distances.distance_diag_left < DISTANCE_WALL_DIAG)
			walls_position.left = WALL_KNOW;
		if (telemeters_distances.distance_diag_right < DISTANCE_WALL_DIAG)
			walls_position.right = WALL_KNOW;
	}
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
		ssd1306Printf(0,0,&Font_5x8,"D f left = %d",(int)(telemeters_distances.distance_front_left));
		ssd1306Printf(0,10,&Font_5x8,"D f right = %d",(int)(telemeters_distances.distance_front_right));
		ssd1306Printf(0,20,&Font_5x8,"D d left = %d",(int)(telemeters_distances.distance_diag_left));
		ssd1306Printf(0,30,&Font_5x8,"D d right = %d",(int)(telemeters_distances.distance_diag_right));
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
		if (wall_see.front == WALL_KNOW)
		{
			ssd1306FillRect(0,49,54,5);
		}
		else
		{
			ssd1306DrawRect(0,49,54,5);
		}
		switch (wall_see.next_front)
		{
			case WALL_KNOW:
				ssd1306FillRect(0,0,54,5);
				break;
			case NO_KNOW :
				ssd1306DrawRect(0,0,54,5);
				break;
			default:
				break;
		}
		switch (wall_see.left)
		{
			case WALL_KNOW:
				ssd1306FillRect(0,0,5,54);
				break;
			case NO_KNOW :
				ssd1306DrawRect(0,0,5,54);
				break;
			default:
				break;
		}
		switch (wall_see.right)
		{
			case WALL_KNOW:
				ssd1306FillRect(49,0,5,54);
				break;
			case NO_KNOW :
				ssd1306DrawRect(49,0,5,54);
				break;
			default:
				break;
		}
		ssd1306Printf(55, 0,&Font_5x8,"F L ;%d",telemeters.left_front.average_value);
		ssd1306Printf(55, 0,&Font_5x8,"F L ;%d",telemeters.left_front.average_value);
		ssd1306Printf(55, 0,&Font_5x8,"F L ;%d",telemeters.left_front.average_value);
		ssd1306Refresh();
	}
	telemetersStop();
}
