/**************************************************************************/
/*!
    @file    telemeters.c5
    @author  PLF (PACABOT)
    @date    11 July 2015
    @version 1.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include "application/statistiques/statistiques.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <string.h>
#include <stdio.h>6
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */

/*application include */
#include "application/solverMaze/solverMaze.h"

/* Declarations for this module */
#include "peripherals/telemeters/telemeters.h"

#include "middleware/wall_sensors/wall_sensors.h"

/* extern variables */
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* global variables */
GPIO_InitTypeDef GPIO_InitStruct;
ADC_ChannelConfTypeDef sConfig;

telemetersStruct telemeters;
extern walls cell_state;

#ifdef DARK
int telemeter_FR_profile[NUMBER_OF_CELL + 1] = {3834,3822,3551,3759,3724,3762,3748,3745,3752,3741,3696,3735,3717,3686,3670,3656,3630,3600,3554,3530,3479,3446,3385,3286,3157,2990,2810,2728,2611,2496,2389,2221,2198,2101,2038,1962,1887,1820,1756,1703,1640,1588,1536,1495,1395,1387,1354,1308,1242,1195,1198,1156,1117,1087,1050,1020,1001,951,936,901,876,846,818,813,778,746,733,712,691,681,666,645,624,616,605,592,573,566,557,546,537,529,521,514,511,496,489,480,471,465,449,450,436,433,420,407,403,401,390,379};
int telemeter_FL_profile[NUMBER_OF_CELL + 1] = {3850,3842,3786,3777,3780,3782,3767,3767,3775,3765,3773,3758,3748,3736,3727,3722,3704,3705,3690,3665,3640,3648,3599,3585,3519,3507,3469,3365,3397,3328,3206,3060,3067,3105,2916,2766,2780,2634,2588,2453,2414,2285,2252,2138,1964,2039,1904,1885,1860,1616,1691,1631,1605,1541,1545,1502,1408,1412,1333,1310,1277,1261,1210,1133,1115,1125,1074,1041,1014,973,949,944,911,901,883,865,857,828,810,797,783,773,760,750,731,724,713,700,689,680,676,640,653,636,625,621,604,596,592,578};
int telemeter_DR_profile[NUMBER_OF_CELL + 1] = {3814,3570,3843,3832,3578,3578,3821,3306,3804,3799,3796,3282,3773,3756,3751,3750,3464,3485,3720,3698,3691,3192,3415,3391,3571,3331,3304,3272,3398,3092,2936,2976,2825,2697,2372,2289,2145,2041,1984,1996,1805,1838,1763,1677,1536,1533,1409,1439,1297,1359,1226,1263,1144,1179,1145,1036,1009,1038,979,907,940,909,863,849,828,804,740,769,749,695,714,678,666,686,635,661,605,617,622,608,593,540,528,548,532,516,500,452,447,464,448,436,415,421,394,409,396,374,371,372};
int telemeter_DL_profile[NUMBER_OF_CELL + 1] = {3863,3850,3865,3844,3843,3827,3826,3814,3813,3799,3805,3794,3779,3780,3764,3769,3762,3758,3748,3743,3735,3731,3727,3722,3714,3708,3711,3703,3693,3688,3675,3670,3653,3644,3630,3612,3587,3564,3547,3511,3490,3451,3420,3397,3325,3290,3180,3049,2961,2850,2868,2717,2629,2542,2474,2402,2309,2233,2179,2061,2009,1938,1859,1836,1788,1741,1686,1637,1574,1585,1481,1455,1423,1405,1351,1338,1314,1306,1286,1245,1162,1204,1177,1153,1121,1088,1069,1048,1025,1002,978,953,932,915,898,871,861,836,845,836};
#else //MEDDLE
int telemeter_FR_profile[NUMBER_OF_CELL + 1] = {4096,3798,3791,3780,3760,3740,3725,3714,3691,3654,3607,3569,3520,3469,3409,3342,3245,3102,2928,2763,2604,2457,2325,2192,2080,1971,1876,1786,1696,1623,1548,1480,1412,1349,1298,1232,1182,1132,1092,1047,997,954,923,888,856,820,792,764,736,721,688,667,655,625,605,584,563,545,529,524,492,487,458,441,431,418,414,389,379,364,356,350,340,328,323,311,304,295,289,281,271,265,259,255,246,240,233,228,224,219,214,205,200,198,196,191,187,185,177,1,0};
int telemeter_FL_profile[NUMBER_OF_CELL + 1] = {4096,3824,3818,3811,3806,3800,3783,3782,3772,3764,3740,3718,3694,3650,3621,3591,3558,3516,3432,3425,3354,3270,3020,2970,2820,2687,2609,2483,2437,2213,2117,2059,2027,1853,1770,1699,1623,1558,1518,1455,1437,1328,1276,1233,1186,1142,1100,1063,1029,996,965,931,928,900,851,822,799,774,746,742,717,680,659,638,619,599,590,572,551,526,504,488,480,475,473,456,434,425,420,415,409,405,395,382,373,365,357,350,343,339,334,329,323,317,308,303,296,289,284,1,0};
int telemeter_DR_profile[NUMBER_OF_CELL + 1] = {4096,3909,3903,3894,3886,3878,3870,3862,3854,3848,3840,3831,3823,3814,3808,3797,3789,3776,3764,3743,3727,3701,3671,3637,3601,3558,3513,3462,3404,3331,3196,3032,2876,2739,2616,2495,2377,2264,2156,2054,1964,1881,1799,1727,1657,1594,1534,1479,1427,1377,1329,1274,1227,1183,1140,1100,1062,1029,997,970,942,918,891,867,844,819,794,770,748,726,706,690,671,654,640,623,610,596,580,564,545,531,517,505,493,482,472,462,453,446,438,430,420,409,397,385,379,370,362,1,0};
int telemeter_DL_profile[NUMBER_OF_CELL + 1] = {4096,3915,3911,3903,3895,3887,3879,3872,3863,3858,3851,3846,3839,3831,3827,3819,3812,3806,3796,3790,3780,3773,3757,3745,3726,3704,3678,3648,3612,3577,3540,3504,3442,3369,3262,3114,2963,2824,2691,2578,2456,2355,2250,2158,2073,1998,1927,1859,1781,1709,1649,1621,1538,1470,1417,1370,1321,1271,1230,1195,1167,1133,1104,1077,1042,1009,977,950,924,894,867,849,830,800,780,763,748,732,710,689,667,656,630,614,597,582,568,558,551,544,535,528,516,502,490,480,474,465,456,1,0};
#endif

/*## ADC Telemeters callback for distance computing  #############################*/
/* ------------------------------------------------------------------------
	    ADC2_IN11	R_DIAG_RX		ADC_REGULAR_RANK_1
	    ADC2_IN6	L_FRONT_RX		ADC_REGULAR_RANK_2
		ADC2_IN10	R_FRONT_RX		ADC_REGULAR_RANK_3
		ADC2_IN5	L_DIAG_RX		ADC_REGULAR_RANK_4
------------------------------------------------------------------------ */

void telemetersInit(void)
{
	//	ADC_InjectionConfTypeDef sConfigInjected;
	HAL_ADC_Stop_IT(&hadc2);
	HAL_ADC_Stop_IT(&hadc3);

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc2);

	hadc3.Instance = ADC3;
	hadc3.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc3.Init.Resolution = ADC_RESOLUTION12b;
	hadc3.Init.ScanConvMode = DISABLE;
	hadc3.Init.ContinuousConvMode = DISABLE;
	hadc3.Init.DiscontinuousConvMode = DISABLE;
	hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc3.Init.NbrOfConversion = 1;
	hadc3.Init.DMAContinuousRequests = DISABLE;
	hadc3.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc3);

	memset(&telemeters.FR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.FL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.DR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters.DL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset(&telemeters, 0, sizeof(telemetersStruct));

	telemeters.FL.mm_conv.old_avrg = 0;
	telemeters.FR.mm_conv.old_avrg = 0;
	telemeters.DL.mm_conv.old_avrg = 0;
	telemeters.DR.mm_conv.old_avrg = 0;

	telemeters.FL.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.DL.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.FR.mm_conv.cell_idx = NUMBER_OF_CELL - 1;
	telemeters.DR.mm_conv.cell_idx = NUMBER_OF_CELL - 1;

	telemeters.FL.mm_conv.profile = telemeter_FL_profile;
	telemeters.DL.mm_conv.profile = telemeter_DL_profile;
	telemeters.FR.mm_conv.profile = telemeter_FR_profile;
	telemeters.DR.mm_conv.profile = telemeter_DR_profile;

	telemeters.FL.led_gpio = TX_FL;
	telemeters.DL.led_gpio = TX_DUAL_DIAG;
	telemeters.FR.led_gpio = TX_FR;
	telemeters.DR.led_gpio = TX_DUAL_DIAG;
}

void telemetersStart(void)
{
	telemeters.active_state = TRUE;
}

void telemetersStop(void)
{
	HAL_GPIO_WritePin(GPIOB, TX_FL, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_FR, RESET);
	HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
	telemeters.active_state = FALSE;
}

void telemetersAdc2Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start_IT(&hadc2);
	telemeters.it_cnt++;
}

void telemetersAdc3Start(void)
{
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc3, &sConfig);
	HAL_ADC_Start_IT(&hadc3);
}

void telemeters_IT(void)
{
	telemeters.selector++;

	if (telemeters.selector > 10)
	{
		telemeters.selector = 0;
		HAL_GPIO_WritePin(GPIOB, TX_FR, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_FL, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, RESET);
		telemeters.FR.isActivated = 0;
		telemeters.FL.isActivated = 0;
		telemeters.DL.isActivated = 0;
		telemeters.DR.isActivated = 0;
	}

	switch (telemeters.selector)
	{
	case 1:
		HAL_GPIO_WritePin(GPIOB, TX_FL, SET);
		return;
	case 2:
		telemeters.FL.isActivated = TX_ON;
		sConfig.Channel = RX_FL;
		telemetersAdc2Start();
		return;
	case 3:
		HAL_GPIO_WritePin(GPIOB, TX_FR, SET);
		return;
	case 4:
		telemeters.FR.isActivated = TX_ON;
		sConfig.Channel = RX_FR;
		telemetersAdc2Start();
		return;
	case 5:
		HAL_GPIO_WritePin(GPIOB, TX_DUAL_DIAG, SET);
		return;
	case 6:
		telemeters.DL.isActivated = TX_ON;
		sConfig.Channel = RX_DL;
		telemetersAdc2Start();
		telemeters.DR.isActivated = TX_ON;
		sConfig.Channel = RX_DR;
		telemetersAdc3Start();
		return;
	case 7:
		return;
	case 8:
		telemeters.FR.isActivated = TX_OFF;
		sConfig.Channel = RX_FR;
		telemetersAdc2Start();
		return;
	case 9:
		telemeters.FL.isActivated = TX_OFF;
		sConfig.Channel = RX_FL;
		telemetersAdc2Start();
		return;
	case 10:
		telemeters.DL.isActivated = TX_OFF;
		sConfig.Channel = RX_DL;
		telemetersAdc2Start();
		telemeters.DR.isActivated = TX_OFF;
		sConfig.Channel = RX_DR;
		telemetersAdc3Start();
		return;

	}
}

void telemeters_ADC2_IT(void)
{
	if (telemeters.FL.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.FL, &hadc2);
		getTelemetersDistance(&telemeters.FL);
		goto end;
	}
	if (telemeters.FR.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.FR, &hadc2);
		getTelemetersDistance(&telemeters.FR);
		goto end;
	}
	if (telemeters.DL.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.DL, &hadc2);
		getTelemetersDistance(&telemeters.DL);
		goto end;
	}

end :
	telemeters.end_of_conversion++;
}

void telemeters_ADC3_IT(void)
{
	if (telemeters.DR.isActivated != FALSE)
	{
		getTelemetersADC(&telemeters.DR, &hadc3);
		getTelemetersDistance(&telemeters.DR);
	}
}

void getTelemetersADC(telemeterStruct *tel, ADC_HandleTypeDef *hadc)
{
	if (tel->isActivated == TX_ON)
	{
		tel->adc = HAL_ADC_GetValue(hadc);
		HAL_GPIO_WritePin(GPIOB, tel->led_gpio, RESET);

		if (tel->adc - tel->avrg_ref > 0)
			tel->avrg = mobileAvrgInt(&tel->mAvrgStruct, (tel->adc - tel->avrg_ref));
		else
			tel->avrg = 0;

		tel->isActivated = FALSE;
	}
	if (tel->isActivated == TX_OFF)
	{
		tel->adc_ref = HAL_ADC_GetValue(hadc);
		tel->avrg_ref = mobileAvrgInt(&tel->mAvrgStruct_ref, tel->adc_ref);

		tel->isActivated = FALSE;
	}
}

/*
 * Formulas
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

void getTelemetersDistance(telemeterStruct *tel)
{
	char sens = 1;

	if(tel->avrg > tel->mm_conv.old_avrg)
	{
		sens = -1;
	}
	else if(tel->avrg == tel->mm_conv.old_avrg)	//for optimize redundant call
	{
		return;
	}

	while ((tel->avrg > tel->mm_conv.profile[tel->mm_conv.cell_idx]) || (tel->avrg < tel->mm_conv.profile[tel->mm_conv.cell_idx + 1]))
	{
		tel->mm_conv.cell_idx += sens;
		if (tel->mm_conv.cell_idx < 0)
		{
			tel->mm_conv.cell_idx = 0;
			break;
		}
		else if (tel->mm_conv.cell_idx >= NUMBER_OF_CELL)
		{
			tel->mm_conv.cell_idx = NUMBER_OF_CELL;
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
	 * XXXX is the sensor reference : front_left , front_right , diag_right , diag_left
	 *
	 *
	 * 		telemeter_XXXX_voltage[cell_XXXX] (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP - cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP telemeter_XXXX_voltage[cell_XXXX + 1] - value_XXX (cell_XXXX+1)*NUMBER_OF_MILLIMETER_BY_LOOP + value_XXXX cell_XXXX*NUMBER_OF_MILLIMETER_BY_LOOP
	 * xc= _______________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________
	 * 																		-telemeter_XXX_voltage[cell_XXXX + 1] + telemeter_XXX_voltage[cell_XXXX]
	 *
	 */
	tel->dist_mm =
			((double)((tel->mm_conv.profile[tel->mm_conv.cell_idx] * (tel->mm_conv.cell_idx + 1) * NUMBER_OF_MILLIMETER_BY_LOOP) -
					(tel->mm_conv.cell_idx * NUMBER_OF_MILLIMETER_BY_LOOP * tel->mm_conv.profile[tel->mm_conv.cell_idx + 1]) -
					(tel->avrg * (tel->mm_conv.cell_idx + 1) * NUMBER_OF_MILLIMETER_BY_LOOP) +
					(tel->avrg * tel->mm_conv.cell_idx * NUMBER_OF_MILLIMETER_BY_LOOP))) /
							((double)(-tel->mm_conv.profile[tel->mm_conv.cell_idx + 1] + (double)tel->mm_conv.profile[tel->mm_conv.cell_idx]));

	tel->delta_avrg = mobileAvrgInt(&tel->sAvrgStruct, (int)((tel->mm_conv.old_dist_mm - tel->dist_mm) * 1000)) / 1000.00;
	tel->speed_mms = tel->delta_avrg * (double)(TELEMETERS_TIME_FREQ / 10.00);

	tel->mm_conv.old_avrg = tel->avrg;
	tel->mm_conv.old_dist_mm = tel->dist_mm;

	if (telemeters.FL.dist_mm < DISTANCE_FIRST_WALL_FRONT)
		cell_state.front = WALL_PRESENCE;
	if (telemeters.FL.dist_mm < DISTANCE_SEGOND_WALL_FRONT)
		cell_state.next_front = WALL_PRESENCE;
	if (telemeters.DL.dist_mm < DISTANCE_WALL_DIAG)
		cell_state.left = WALL_PRESENCE;
	if (telemeters.DR.dist_mm < DISTANCE_WALL_DIAG)
		cell_state.right = WALL_PRESENCE;
}

int getTelemetersVariation(telemeterStruct *tel)
{
	getTelemetersDistance(tel);
	return tel->delta_mm;
}

void telemetersTest(void)
{
	char joy;
	telemetersInit();
	telemetersStart();

	while (joy != JOY_LEFT)	//todo make a generic test menu (unit test)
	{
		joy = expanderJoyFiltered();
		ssd1306ClearScreen();

		ssd1306DrawString(1,0, "   AVRG 1/10mm", &Font_5x8);

		ssd1306PrintInt(1, 9 , "FL ", (int32_t) telemeters.FL.avrg, &Font_5x8);
		ssd1306PrintInt(1, 18, "DL ", (int32_t) telemeters.DL.avrg, &Font_5x8);
		ssd1306PrintInt(1, 27, "DR ", (int32_t) telemeters.DR.avrg, &Font_5x8);
		ssd1306PrintInt(1, 36, "FR ", (int32_t) telemeters.FR.avrg, &Font_5x8);

		ssd1306PrintInt(45, 9 ,"", (int32_t) (telemeters.FL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 18,"", (int32_t) (telemeters.DL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 27,"", (int32_t) (telemeters.DR.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 36,"", (int32_t) (telemeters.FR.dist_mm * 10), &Font_5x8);

//		ssd1306PrintInt(1, 45, "IT TIME  =  ", (int32_t) telemeters.it_time, &Font_5x8);
		ssd1306PrintInt(1, 54, "IT ERROR =  ", (telemeters.it_cnt - telemeters.end_of_conversion), &Font_5x8);
		ssd1306Refresh();

		if (joy == JOY_RIGHT)
		{
			while (joy != JOY_LEFT)
			{
				joy = expanderJoyFiltered();
				ssd1306ClearScreen();
				ssd1306DrawString(1,0, "   ADC  REF  VAR", &Font_5x8);

				ssd1306PrintInt(1, 9 , "FL ", (int32_t) telemeters.FL.adc, &Font_5x8);
				ssd1306PrintInt(1, 18, "DL ", (int32_t) telemeters.DL.adc, &Font_5x8);
				ssd1306PrintInt(1, 27, "DR ", (int32_t) telemeters.DR.adc, &Font_5x8);
				ssd1306PrintInt(1, 36, "FR ", (int32_t) telemeters.FR.adc, &Font_5x8);

				ssd1306PrintInt(45, 9 , "", (int32_t) telemeters.FL.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 18, "", (int32_t) telemeters.DL.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 27, "", (int32_t) telemeters.DR.avrg_ref, &Font_5x8);
				ssd1306PrintInt(45, 36, "", (int32_t) telemeters.FR.avrg_ref, &Font_5x8);

//				ssd1306PrintInt(75, 9 , "", (int32_t) (telemeters.FL.delta_avrg), &Font_5x8);
//				ssd1306PrintInt(75, 18, "", (int32_t) (telemeters.DL.delta_avrg), &Font_5x8);
//				ssd1306PrintInt(75, 27, "", (int32_t) telemeters.DR.delta_mm_avrg, &Font_5x8);
//				ssd1306PrintInt(75, 36, "", (int32_t) telemeters.FR.delta_mm_avrg, &Font_5x8);

				ssd1306PrintInt(75, 9 , "", (int32_t) telemeters.FL.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 18, "", (int32_t) telemeters.DL.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 27, "", (int32_t) telemeters.DR.speed_mms, &Font_5x8);
				ssd1306PrintInt(75, 36, "", (int32_t) telemeters.FR.speed_mms, &Font_5x8);

				ssd1306Refresh();
			}
			while (joy == JOY_LEFT)
			{
				joy = expanderJoyFiltered();
			}
		}
	}
	antiBounceJoystick();
	telemetersStop();
}
