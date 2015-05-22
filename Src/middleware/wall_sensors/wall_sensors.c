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
int telemeter_right_front_voltage[NUMBER_OF_CELL]={3854,3840,3831,3821,3812,3805,3798,3762,3754,3710,3661,3626,3589,3540,3488,3424,3327,3142,2964,2850,2659,2518,2377,2251,2136,2020,1922,1834,1750,1720,1570,1539,1476,1423,1368,1321,1274,1221,1180,1132,1086,1053,1014,979,945,915,885,860,831,800,778,751,733,707,685,659,638,616,597,580,559,544,518,512,492,478,468,456,448,436,427,420,407,400,392,382,373,362,355,345,337,328,322,316,309,302,298,286,281,277,270,261,256,250,241,237,232,224,220,212};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={3850,3842,3786,3777,3780,3782,3767,3767,3775,3765,3773,3758,3748,3736,3727,3722,3704,3705,3690,3665,3640,3648,3599,3585,3519,3507,3469,3365,3397,3328,3206,3060,3067,3105,2916,2766,2780,2634,2588,2453,2414,2285,2252,2138,1964,2039,1904,1885,1860,1616,1691,1631,1605,1541,1545,1502,1408,1412,1333,1310,1277,1261,1210,1133,1115,1125,1074,1041,1014,973,949,944,911,901,883,865,857,828,810,797,783,773,760,750,731,724,713,700,689,680,676,640,653,636,625,621,604,596,592,578};
int telemeter_right_diag_voltage[NUMBER_OF_CELL]={3814,3570,3843,3832,3578,3578,3821,3306,3804,3799,3796,3282,3773,3756,3751,3750,3464,3485,3720,3698,3691,3192,3415,3391,3571,3331,3304,3272,3398,3092,2936,2976,2825,2697,2372,2289,2145,2041,1984,1996,1805,1838,1763,1677,1536,1533,1409,1439,1297,1359,1226,1263,1144,1179,1145,1036,1009,1038,979,907,940,909,863,849,828,804,740,769,749,695,714,678,666,686,635,661,605,617,622,608,593,540,528,548,532,516,500,452,447,464,448,436,415,421,394,409,396,374,371,372};
int telemeter_left_diag_voltage[NUMBER_OF_CELL]={3863,3850,3865,3844,3843,3827,3826,3814,3813,3799,3805,3794,3779,3780,3764,3769,3762,3758,3748,3743,3735,3731,3727,3722,3714,3708,3711,3703,3693,3688,3675,3670,3653,3644,3630,3612,3587,3564,3547,3511,3490,3451,3420,3397,3325,3290,3180,3049,2961,2850,2868,2717,2629,2542,2474,2402,2309,2233,2179,2061,2009,1938,1859,1836,1788,1741,1686,1637,1574,1585,1481,1455,1423,1405,1351,1338,1314,1306,1286,1245,1162,1204,1177,1153,1121,1088,1069,1048,1025,1002,978,953,932,915,898,871,861,836,845,836};
#else
int telemeter_right_front_voltage[NUMBER_OF_CELL]={4096,3800,3774,3758,3700,3690,3639,3535,3520,3486,3414,3321,3167,2966,2777,2605,2444,2300,2155,2033,1907,1811,1712,1619,1533,1455,1377,1314,1260,1209,1162,1108,1062,1022,985,951,916,881,847,816,782,755,727,702,677,657,636,614,596,577,558,535,523,502,486,464,446,430,420,400,391,375,361,341,337,320,317,310,301,296,289,283,275,271,263,256,251,243,237,230,225,218,214,209,204,202,195,189,185,179,177,172,166,160,155,151,145,140,138,0};
int telemeter_left_front_voltage[NUMBER_OF_CELL]={4096,3840,3831,3821,3812,3805,3798,3762,3754,3710,3661,3626,3589,3540,3488,3424,3327,3142,2964,2850,2659,2518,2377,2251,2136,2020,1922,1834,1750,1720,1570,1539,1476,1423,1368,1321,1274,1221,1180,1132,1086,1053,1014,979,945,915,885,860,831,800,778,751,733,707,685,659,638,616,597,580,559,544,518,512,492,478,468,456,448,436,427,420,407,400,392,382,373,362,355,345,337,328,322,316,309,302,298,286,281,277,270,261,256,250,241,237,232,224,220,0};
int telemeter_right_diag_voltage[NUMBER_OF_CELL]={4096,3907,3897,3891,3883,3876,3866,3860,3850,3840,3829,3816,3809,3792,3773,3743,3711,3668,3620,3560,3504,3443,3360,3215,3019,2847,2687,2531,2387,2251,2135,2013,1905,1801,1709,1616,1529,1451,1382,1317,1252,1190,1132,1081,1038,994,954,919,883,849,818,783,754,725,700,672,645,619,593,574,550,531,509,488,466,446,427,408,391,372,354,341,330,319,309,298,289,280,271,262,253,244,235,226,218,213,205,198,192,185,179,171,163,158,148,137,136,134,126,0};
int telemeter_left_diag_voltage[NUMBER_OF_CELL]={4096,3929,3926,3919,3914,3901,3894,3886,3881,3869,3867,3853,3851,3838,3828,3819,3808,3801,3788,3763,3738,3719,3686,3656,3617,3580,3529,3478,3400,3269,3119,2958,2817,2674,2548,2427,2312,2206,2108,2014,1922,1834,1747,1669,1601,1533,1473,1417,1362,1312,1265,1218,1182,1140,1104,1067,1034,1001,964,938,907,880,851,824,794,766,739,713,688,665,642,621,603,585,567,551,537,523,509,494,481,470,459,447,436,428,418,407,397,389,381,371,361,353,343,329,328,322,314,0};
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

	mainControlInit();
	telemetersInit();
	telemetersStart();
	motorsSleepDriver(OFF);

	control_params.speed_state = TRUE;
	control_params.follow_state = FALSE;
	control_params.position_state = TRUE;
	HAL_Delay(1000);
//	for(int i = 0; i < NUMBER_OF_CELL; i++)
//	{
//		ssd1306ProgressBar(10,10,(i*100)/NUMBER_OF_CELL);
//		ssd1306ProgressBar(10,40,(i*50)/NUMBER_OF_CELL);
//		ssd1306Refresh();
//
//		telemeter_left_front_voltage[i]=telemeters.left_front.average_value;
//		telemeter_right_front_voltage[i]=telemeters.right_front.average_value;
//
//		move(0,-NUMBER_OF_MILLIMETER_BY_LOOP,50,0);
//		while(speed_control.end_control != 1);
//		HAL_Delay(500);
//	}
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
		telemeter_left_diag_voltage[i]=telemeters.left_diag.average_value;
		telemeter_right_diag_voltage[i]=telemeters.right_diag.average_value;
		move(0,-sqrtf(2.00*powf(NUMBER_OF_MILLIMETER_BY_LOOP,2)),50,0);
		while(isEndMove() != 1);
		HAL_Delay(500);
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
			cell_front_left=NUMBER_OF_CELL - 1;
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
			cell_front_right=NUMBER_OF_CELL - 1;
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
			cell_diag_left=NUMBER_OF_CELL - 1;
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
			cell_diag_right=NUMBER_OF_CELL - 1;
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
	if ((telemeters_distances.distance_front_left < DISTANCE_FIRST_WALL_FRONT) || (telemeters_distances.distance_front_right< DISTANCE_FIRST_WALL_FRONT))
	{
		walls_position.front = WALL_PRESENCE;
	}
	if (telemeters_distances.distance_front_left < DISTANCE_SEGOND_WALL_FRONT)
		walls_position.next_front = WALL_PRESENCE;
	if (telemeters_distances.distance_diag_left < DISTANCE_WALL_DIAG)
		walls_position.left = WALL_PRESENCE;
	if (telemeters_distances.distance_diag_right < DISTANCE_WALL_DIAG)
		walls_position.right = WALL_PRESENCE;
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
	telemetersStart();
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
