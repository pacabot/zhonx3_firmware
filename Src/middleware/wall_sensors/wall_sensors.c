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
int telemeter_right_front_voltage[NUMBER_OF_CELL]={4996,3798,3791,3780,3760,3740,3725,3714,3691,3654,3607,3569,3520,3469,3409,3342,3245,3102,2928,2763,2604,2457,2325,2192,2080,1971,1876,1786,1696,1623,1548,1480,1412,1349,1298,1232,1182,1132,1092,1047,997,954,923,888,856,820,792,764,736,721,688,667,655,625,605,584,563,545,529,524,492,487,458,441,431,418,414,389,379,364,356,350,340,328,323,311,304,295,289,281,271,265,259,255,246,240,233,228,224,219,214,205,200,198,196,191,187,185,177,0};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={4996,3824,3818,3811,3806,3800,3783,3782,3772,3764,3740,3718,3694,3650,3621,3591,3558,3516,3432,3425,3354,3270,3020,2970,2820,2687,2609,2483,2437,2213,2117,2059,2027,1853,1770,1699,1623,1558,1518,1455,1437,1328,1276,1233,1186,1142,1100,1063,1029,996,965,931,928,900,851,822,799,774,746,742,717,680,659,638,619,599,590,572,551,526,504,488,480,475,473,456,434,425,420,415,409,405,395,382,373,365,357,350,343,339,334,329,323,317,308,303,296,289,284,0};
int telemeter_right_diag_voltage[NUMBER_OF_CELL]={4996,3909,3903,3894,3886,3878,3870,3862,3854,3848,3840,3831,3823,3814,3808,3797,3789,3776,3764,3743,3727,3701,3671,3637,3601,3558,3513,3462,3404,3331,3196,3032,2876,2739,2616,2495,2377,2264,2156,2054,1964,1881,1799,1727,1657,1594,1534,1479,1427,1377,1329,1274,1227,1183,1140,1100,1062,1029,997,970,942,918,891,867,844,819,794,770,748,726,706,690,671,654,640,623,610,596,580,564,545,531,517,505,493,482,472,462,453,446,438,430,420,409,397,385,379,370,362,0};
int telemeter_left_diag_voltage[NUMBER_OF_CELL]={4996,3915,3911,3903,3895,3887,3879,3872,3863,3858,3851,3846,3839,3831,3827,3819,3812,3806,3796,3790,3780,3773,3757,3745,3726,3704,3678,3648,3612,3577,3540,3504,3442,3369,3262,3114,2963,2824,2691,2578,2456,2355,2250,2158,2073,1998,1927,1859,1781,1709,1649,1621,1538,1470,1417,1370,1321,1271,1230,1195,1167,1133,1104,1077,1042,1009,977,950,924,894,867,849,830,800,780,763,748,732,710,689,667,656,630,614,597,582,568,558,551,544,535,528,516,502,490,480,474,465,456,0};
#endif
//int telemetersWithOutNoise//todo
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
	control_params.speed_state = TRUE;
	control_params.follow_state = FALSE;
	control_params.position_state = TRUE;
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
	do
	{
		while (length_front_right>0 || length_front_left>0)
		{
			if(length_front_left>=0 && \
					((position_zhonx-value_to_retest_front_left[length_front_left]) < (position_zhonx-value_to_retest_front_right[length_front_right])\
					|| length_front_right<=0))
			{
				distance=(position_zhonx-value_to_retest_front_left[length_front_left])*NUMBER_OF_MILLIMETER_BY_LOOP;
				bluetoothPrintf("position Zhonx=%d,\tvaleur gauche a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_left[length_front_left], distance);
				if (distance!=0)
				{
					move(0,(float)distance,10,0);
					while(speed_control.end_control != 1);
					position_zhonx=value_to_retest_front_left[length_front_left];
				}

				telemeter_left_front_voltage[value_to_retest_front_left[length_front_left]]=telemeters.left_front.average_value;

				length_front_left--;
			}
			else if (length_front_right>=0)
			{
				distance=(position_zhonx-value_to_retest_front_right[length_front_right])*NUMBER_OF_MILLIMETER_BY_LOOP;
				bluetoothPrintf("position Zhonx=%d,\tvaleur droite a retester=%d\t diff=%d\n", position_zhonx, value_to_retest_front_right[length_front_right],distance);
				if(distance!=0)
				{
					move(0,distance,10,0);
					while(speed_control.end_control != 1);
					position_zhonx=value_to_retest_front_right[length_front_right];
				}
				telemeter_right_front_voltage[value_to_retest_front_right[length_front_right]]=telemeters.right_front.average_value;

				length_front_right--;
			}
		}
		bluetoothPrintf("\n\n\nfilterd measures :\n");
		for (int i = 0; i < NUMBER_OF_CELL; ++i)
		{
			bluetoothPrintf("%2d|%10d|%d\n",i,telemeter_left_front_voltage[i],telemeter_right_front_voltage[i]);
		}
		length_front_left=0;
		length_front_right=0;
		for(int i = 1; i<(NUMBER_OF_CELL-1); i++)
		{
			if (telemeter_left_front_voltage[i] <= telemeter_left_front_voltage[i+1])
			{
				value_to_retest_front_left[length_front_left] = i;
				length_front_left ++;
				if( i==(NUMBER_OF_CELL-2) )
				{
					value_to_retest_front_left[length_front_left] = i+1;
					length_front_left ++;
				}
			}
			if (telemeter_right_front_voltage[i] <= telemeter_right_front_voltage[i+1])
			{
				value_to_retest_front_right[length_front_right] = i;
				length_front_right ++;
				if( i==(NUMBER_OF_CELL-2) )
				{
					value_to_retest_front_right[length_front_right] = i+1;
					length_front_right ++;
				}
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
		bluetoothPrintf("nombre de valeur a retester : a gauche : %d, a d)roite %d\n",length_front_left,length_front_right);
	}while (length_front_right>0 || length_front_left>0);

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
 * Formula
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
	static int old_value_diag_right	= 0;
	static int old_value_diag_left	= 0;
	static int old_value_front_right	= 0;
	static int old_value_front_left	= 0;

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

	if(value_front_left > old_value_front_left)
	{
		sens_front_left=-1;
	}
	if(value_front_right > old_value_front_right)
	{
		sens_front_right=-1;
	}
	if(value_diag_left > old_value_diag_left)
	{
		sens_diag_left=-1;
	}
	if(value_diag_right > old_value_diag_right)
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
	old_value_front_left = value_front_left;

	telemeters_distances->distance_front_right = (telemeter_right_front_voltage[cell_front_right] * (cell_front_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_front_right*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_right_front_voltage[cell_front_right + 1] - value_front_right * (cell_front_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_front_right * cell_front_right*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_right_front_voltage[cell_front_right + 1] + telemeter_right_front_voltage[cell_front_right]);
	old_value_front_right = value_front_right;

	telemeters_distances->distance_diag_left = (telemeter_left_diag_voltage[cell_diag_left] * (cell_diag_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_diag_left*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_left_diag_voltage[cell_diag_left + 1] - value_diag_left * (cell_diag_left+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_diag_left * cell_diag_left*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_left_diag_voltage[cell_diag_left + 1] + telemeter_left_diag_voltage[cell_diag_left]);
	old_value_diag_left = value_diag_left;

	telemeters_distances->distance_diag_right = (telemeter_right_diag_voltage[cell_diag_right] * (cell_diag_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_diag_right*NUMBER_OF_MILLIMETER_BY_LOOP * telemeter_right_diag_voltage[cell_diag_right + 1] - value_diag_right * (cell_diag_right+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_diag_right * cell_diag_right*NUMBER_OF_MILLIMETER_BY_LOOP)/(-telemeter_right_diag_voltage[cell_diag_right + 1] + telemeter_right_diag_voltage[cell_diag_right]);
	old_value_diag_right = value_diag_right;
	return WALL_SENSORS_E_SUCCESS;
}

walls getCellState()
{
	telemetersDistancesTypeDef telemeters_distances;
	walls walls_position = {NO_WALL,NO_WALL,NO_WALL,NO_WALL};

	getTelemetersDistance(&telemeters_distances);
	if (telemeters_distances.distance_front_left < DISTANCE_FIRST_WALL_FRONT)
	{
		walls_position.front = WALL_PRESENCE;

		walls_position.next_front = NO_KNOWN;
		walls_position.left = NO_KNOWN;
		walls_position.right = NO_KNOWN;
	}
	else
	{
		if (telemeters_distances.distance_front_left < DISTANCE_SEGOND_WALL_FRONT)
			walls_position.next_front = WALL_PRESENCE;
		if (telemeters_distances.distance_diag_left < DISTANCE_WALL_DIAG)
			walls_position.left = WALL_PRESENCE;
		if (telemeters_distances.distance_diag_right < DISTANCE_WALL_DIAG)
			walls_position.right = WALL_PRESENCE;
	}
	return walls_position;
}

void testTelemeterDistance()
{
	telemetersDistancesTypeDef telemeters_distances;
	telemetersInit();
	telemetersStart();

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		ssd1306ClearScreen();
		getTelemetersDistance(&telemeters_distances);
		ssd1306Printf(0,0,&Font_5x8,"D f l=%d",(int)(telemeters_distances.distance_front_left));
		ssd1306Printf(0,10,&Font_5x8,"D f r=%d",(int)(telemeters_distances.distance_front_right));
		ssd1306Printf(0,20,&Font_5x8,"D d l=%d",(int)(telemeters_distances.distance_diag_left));
		ssd1306Printf(0,30,&Font_5x8,"D d r=%d",(int)(telemeters_distances.distance_diag_right));

		ssd1306Printf(60,0,&Font_5x8,"V f l=%d",(int)(telemeters.left_front.average_value));
		ssd1306Printf(60,10,&Font_5x8,"V f r=%d",(int)(telemeters.right_front.average_value));
		ssd1306Printf(60,20,&Font_5x8,"V d l=%d",(int)(telemeters.left_diag.average_value));
		ssd1306Printf(60,30,&Font_5x8,"V d r=%d",(int)(telemeters.right_diag.average_value));


		ssd1306Refresh();
		//bluetoothPrintf("distance left :%d\tright : %d\n",(int)telemeters_distances.distance_front_left,(int)telemeters_distances.distance_front_right);
	}
	telemetersStop();
}

void testWallsSensors()
{
	telemetersInit();
	walls wall_saw;
	telemetersDistancesTypeDef distances;
	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		getTelemetersDistance(&distances);
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
		ssd1306Printf(55, 0,&Font_5x8,"F L :%d",(int)distances.distance_front_left);
		ssd1306Printf(55, 10,&Font_5x8,"D L :%d",(int)distances.distance_diag_left);
		ssd1306Printf(55, 20,&Font_5x8,"D R :%d",(int)	distances.distance_diag_right);
		ssd1306Refresh();
	}
	telemetersStop();
}
