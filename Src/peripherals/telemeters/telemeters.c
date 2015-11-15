/**************************************************************************/
/*!
    @file    telemeters.c
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
#include <stdio.h>
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

#ifdef DARK
uint16_t telemeter_FR_profile[NUMBER_OF_CELL + 1] = {3834,3822,3551,3759,3724,3762,3748,3745,3752,3741,3696,3735,3717,3686,3670,3656,3630,3600,3554,3530,3479,3446,3385,3286,3157,2990,2810,2728,2611,2496,2389,2221,2198,2101,2038,1962,1887,1820,1756,1703,1640,1588,1536,1495,1395,1387,1354,1308,1242,1195,1198,1156,1117,1087,1050,1020,1001,951,936,901,876,846,818,813,778,746,733,712,691,681,666,645,624,616,605,592,573,566,557,546,537,529,521,514,511,496,489,480,471,465,449,450,436,433,420,407,403,401,390,379};
uint16_t telemeter_FL_profile[NUMBER_OF_CELL + 1] = {3850,3842,3786,3777,3780,3782,3767,3767,3775,3765,3773,3758,3748,3736,3727,3722,3704,3705,3690,3665,3640,3648,3599,3585,3519,3507,3469,3365,3397,3328,3206,3060,3067,3105,2916,2766,2780,2634,2588,2453,2414,2285,2252,2138,1964,2039,1904,1885,1860,1616,1691,1631,1605,1541,1545,1502,1408,1412,1333,1310,1277,1261,1210,1133,1115,1125,1074,1041,1014,973,949,944,911,901,883,865,857,828,810,797,783,773,760,750,731,724,713,700,689,680,676,640,653,636,625,621,604,596,592,578};
uint16_t telemeter_DR_profile[NUMBER_OF_CELL + 1] = {3814,3570,3843,3832,3578,3578,3821,3306,3804,3799,3796,3282,3773,3756,3751,3750,3464,3485,3720,3698,3691,3192,3415,3391,3571,3331,3304,3272,3398,3092,2936,2976,2825,2697,2372,2289,2145,2041,1984,1996,1805,1838,1763,1677,1536,1533,1409,1439,1297,1359,1226,1263,1144,1179,1145,1036,1009,1038,979,907,940,909,863,849,828,804,740,769,749,695,714,678,666,686,635,661,605,617,622,608,593,540,528,548,532,516,500,452,447,464,448,436,415,421,394,409,396,374,371,372};
uint16_t telemeter_DL_profile[NUMBER_OF_CELL + 1] = {3863,3850,3865,3844,3843,3827,3826,3814,3813,3799,3805,3794,3779,3780,3764,3769,3762,3758,3748,3743,3735,3731,3727,3722,3714,3708,3711,3703,3693,3688,3675,3670,3653,3644,3630,3612,3587,3564,3547,3511,3490,3451,3420,3397,3325,3290,3180,3049,2961,2850,2868,2717,2629,2542,2474,2402,2309,2233,2179,2061,2009,1938,1859,1836,1788,1741,1686,1637,1574,1585,1481,1455,1423,1405,1351,1338,1314,1306,1286,1245,1162,1204,1177,1153,1121,1088,1069,1048,1025,1002,978,953,932,915,898,871,861,836,845,836};
#else //MEDDLE
int telemeter_FR_profile[NUMBER_OF_CELL + 1] = {3809,3808,3761,3784,3764,3736,3492,3605,3620,3550,3516,3456,3397,3308,3010,2973,2789,2559,2457,2206,2179,2062,1941,1714,1740,1649,1565,1484,1399,1338,1271,1209,1154,1011,1047,958,952,908,867,831,792,764,735,686,675,648,568,605,557,562,545,497,509,489,473,456,425,420,398,400,388,376,355,341,341,331,318,309,301,289,280,270,234,253,244,237,230,222,214,209,202,196,182,187,183,178,162,158,167,162,146,156,154,138,134,139,127,134,130,128,124,124,118,105,113,98,106,104,103,86,95,91,88,85,85,80,79,75,73,69,63,68,67,68,73,65,63,64,62,60,60,58,57,65,56,54,54,52,45,59,51,49,46,46,44,43,43,39,39,39};
int telemeter_FL_profile[NUMBER_OF_CELL + 1] = {3853,3839,3830,3824,3816,3808,3791,3785,3766,3738,3719,3666,3655,3623,3581,3544,3481,3352,3329,3283,3120,2957,2754,2656,2515,2389,2268,2157,2052,1953,1857,1767,1686,1611,1535,1469,1405,1346,1290,1250,1190,1141,1099,1056,1018,982,946,915,856,853,826,799,766,720,723,701,661,657,637,616,599,574,564,537,531,515,501,484,470,456,427,427,415,386,391,381,370,354,339,340,331,312,314,308,301,296,289,283,277,271,266,261,252,248,242,237,232,227,221,217,214,214,204,200,195,190,185,178,178,169,163,159,155,150,145,141,138,137,130,128,124,126,121,120,121,113,115,115,100,110,99,101,102,105,100,97,94,95,91,90,86,85,84,80,73,73,71,70,67,66};
int telemeter_DR_profile[NUMBER_OF_CELL + 1] = {3919,3918,3911,3903,3895,3887,3877,3870,3863,3846,3848,3838,3828,3797,3811,3800,3768,3763,3736,3708,3664,3635,3594,3547,3436,3445,3376,3269,3108,2948,2775,2618,2484,2357,2243,2043,2007,1911,1816,1726,1642,1566,1497,1427,1364,1299,1250,1193,1140,1093,1052,1007,969,930,895,862,823,788,761,728,705,679,651,630,607,584,563,547,528,511,492,473,448,438,421,408,389,381,367,353,334,332,323,316,297,297,285,280,273,265,260,240,244,236,229,223,219,211,204,198,191,186,177,173,166,164,156,152,148,146,127,140,137,132,130,126,110,122,109,102,110,109,106,104,100,97,94,91,77,82,81,80,77,76,72,72,62,67,66,63,60,61,54,59,51,55,56,55,51,49};
int telemeter_DL_profile[NUMBER_OF_CELL + 1] = {3954,3940,3934,3922,3910,3903,3897,3885,3878,3859,3861,3855,3851,3816,3835,3826,3809,3811,3799,3787,3765,3756,3725,3696,3580,3639,3602,3554,3522,3482,3417,3314,3170,3036,2885,2586,2625,2507,2395,2275,2161,2075,1986,1896,1827,1745,1668,1594,1534,1524,1429,1366,1319,1284,1240,1200,1159,1121,1089,1032,1017,998,966,940,909,884,854,828,799,792,750,721,686,683,664,646,629,630,619,601,574,577,567,552,536,540,522,516,511,502,487,468,473,464,451,438,432,428,421,410,402,398,390,387,380,368,365,357,355,351,316,340,335,331,327,322,285,320,297,284,308,303,298,296,293,287,285,283,245,271,270,268,262,258,259,257,225,251,247,246,242,236,215,237,215,229,233,232,229,222};
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
		ssd1306ClearScreen(MAIN_AERA);

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
		ssd1306Refresh(MAIN_AERA);

		if (joy == JOY_RIGHT)
		{
			while (joy != JOY_LEFT)
			{
				joy = expanderJoyFiltered();
				ssd1306ClearScreen(MAIN_AERA);
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

				ssd1306Refresh(MAIN_AERA);
			}
			while (joy == JOY_LEFT)
			{
				joy = expanderJoyFiltered();
			}
		}
	}
	telemetersStop();
}
