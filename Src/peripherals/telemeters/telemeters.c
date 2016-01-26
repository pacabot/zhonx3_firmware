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
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */

/*application include */
#include "application/solverMaze/solverMaze.h"

/* Declarations for this module */
#include "peripherals/telemeters/telemeters.h"

#include "middleware/wall_sensors/wall_sensors.h"

#define TX_ON			1
#define TX_OFF			2

//#define MIN_TELEMETERS_SPEED 20

#if (MEASURED_DISTANCE)%(TELEMETER_PROFILE_ARRAY_LENGTH) != 0
#error  MEASURED_DISTANCE of cell must be a multiple of NUMBER_OF_CELL
#endif

/* Definition for ADCx Channel Pin */
#define TX_FL				GPIO_PIN_1
#define TX_FR				GPIO_PIN_10
#define TX_DUAL_DIAG		GPIO_PIN_11

/* Definition for ADCx's Channel */
#define RX_FL				ADC_CHANNEL_6
#define RX_DL				ADC_CHANNEL_5
#define RX_DR				ADC_CHANNEL_10
#define RX_FR				ADC_CHANNEL_11

// Declare telemeters profiles in Flash memory
TELEMETERS_PROFILE *telemeters_profile = (TELEMETERS_PROFILE *)ADDR_FLASH_SECTOR_10;


/* Types definitions */

enum telemeterType {FL, DL, DR, FR};

typedef struct
{
	double old_dist_mm;
	int old_avrg;
	int cell_idx;
	int	*profile;
} telemeterConvStruct;

typedef struct
{
	double dist_mm;
	double delta_mm;
	double delta_avrg;
	double speed_mms;
	telemeterConvStruct mm_conv;
	int adc;
	int adc_ref;
	mobileAvrgStruct sAvrgStruct;
	mobileAvrgStruct mAvrgStruct;
	mobileAvrgStruct mAvrgStruct_ref;
	int avrg;
	int avrg_ref;
	char isActivated;
	uint16_t led_gpio;
} telemeterStruct;

typedef struct
{
	telemeterStruct FR;
	telemeterStruct DR;
	telemeterStruct FL;
	telemeterStruct DL;

	int it_cnt;
	int end_of_conversion;
	char active_state;
	char selector;
} telemetersStruct;

static volatile telemetersStruct telemeters;

GPIO_InitTypeDef GPIO_InitStruct;
ADC_ChannelConfTypeDef sConfig;

/* Static functions */
static void   telemetersAdc2Start(void);
static void   telemetersAdc3Start(void);
static void   getTelemetersADC(telemeterStruct *tel, ADC_HandleTypeDef *hadc);
static void   getTelemetersDistance (telemeterStruct *tel);
static int	  getTelemetersVariation(telemeterStruct *tel);

/* extern variables */
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

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

	memset((telemeterStruct*)&telemeters.FR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset((telemeterStruct*)&telemeters.FL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset((telemeterStruct*)&telemeters.DR.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset((telemeterStruct*)&telemeters.DL.mAvrgStruct, 0, sizeof(mobileAvrgStruct));
	memset((telemeterStruct*)&telemeters, 0, sizeof(telemetersStruct));

	telemeters.FL.mm_conv.old_avrg = 0;
	telemeters.FR.mm_conv.old_avrg = 0;
	telemeters.DL.mm_conv.old_avrg = 0;
	telemeters.DR.mm_conv.old_avrg = 0;

	telemeters.FL.mm_conv.cell_idx = TELEMETER_PROFILE_ARRAY_LENGTH - 1;
	telemeters.DL.mm_conv.cell_idx = TELEMETER_PROFILE_ARRAY_LENGTH - 1;
	telemeters.FR.mm_conv.cell_idx = TELEMETER_PROFILE_ARRAY_LENGTH - 1;
	telemeters.DR.mm_conv.cell_idx = TELEMETER_PROFILE_ARRAY_LENGTH - 1;

//	telemeters.FL.mm_conv.profile = telemeter_FL_profile;
//	telemeters.DL.mm_conv.profile = telemeter_DL_profile;
//	telemeters.FR.mm_conv.profile = telemeter_FR_profile;
//	telemeters.DR.mm_conv.profile = telemeter_DR_profile;

	telemeters.FL.mm_conv.profile = telemeters_profile->front.left;
    telemeters.FR.mm_conv.profile = telemeters_profile->front.right;
    telemeters.DL.mm_conv.profile = telemeters_profile->diag.left;
    telemeters.DR.mm_conv.profile = telemeters_profile->diag.right;

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

double getTelemeterDist(enum telemeterName telemeter_name)
{
	switch (telemeter_name)
	{
	case TELEMETER_DL:
		return telemeters.DL.dist_mm;
	case TELEMETER_DR:
		return telemeters.DR.dist_mm;
	case TELEMETER_FL:
		return telemeters.FL.dist_mm;
	case TELEMETER_FR:
		return telemeters.FR.dist_mm;
	}
	return 0.00; //todo return correct error ID
}

int getTelemeterAvrg(enum telemeterName telemeter_name)
{
	switch (telemeter_name)
	{
        case TELEMETER_DL:
            return telemeters.DL.avrg;
        case TELEMETER_DR:
            return telemeters.DR.avrg;
        case TELEMETER_FL:
            return telemeters.FL.avrg;
        case TELEMETER_FR:
            return telemeters.FR.avrg;
	}
	return 0; //todo return correct error ID
}

double getTelemeterSpeed(enum telemeterName telemeter_name)
{
	switch (telemeter_name)
	{
	case TELEMETER_DL: //todo
		return 0;
	case TELEMETER_DR:
		return 0;
	case TELEMETER_FL:
		return 0;
	case TELEMETER_FR:
		return 0;
	}
	return 0.00; //todo return correct error ID
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
	if (telemeters.active_state == FALSE)
		return;

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
		getTelemetersADC((telemeterStruct*)&telemeters.FL, &hadc2);
		getTelemetersDistance((telemeterStruct*)&telemeters.FL);
		goto end;
	}
	if (telemeters.FR.isActivated != FALSE)
	{
		getTelemetersADC((telemeterStruct*)&telemeters.FR, &hadc2);
		getTelemetersDistance((telemeterStruct*)&telemeters.FR);
		goto end;
	}
	if (telemeters.DL.isActivated != FALSE)
	{
		getTelemetersADC((telemeterStruct*)&telemeters.DL, &hadc2);
		getTelemetersDistance((telemeterStruct*)&telemeters.DL);
		goto end;
	}

	end :
	telemeters.end_of_conversion++;
}

void telemeters_ADC3_IT(void)
{
	if (telemeters.DR.isActivated != FALSE)
	{
		getTelemetersADC((telemeterStruct*)&telemeters.DR, &hadc3);
		getTelemetersDistance((telemeterStruct*)&telemeters.DR);
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

	while ((tel->avrg > tel->mm_conv.profile[tel->mm_conv.cell_idx]) ||
	       (tel->avrg < tel->mm_conv.profile[tel->mm_conv.cell_idx + 1]))
	{
		tel->mm_conv.cell_idx += sens;
		if (tel->mm_conv.cell_idx < 0)
		{
			tel->mm_conv.cell_idx = 0;
			break;
		}
		else if (tel->mm_conv.cell_idx >= TELEMETER_PROFILE_ARRAY_LENGTH)
		{
			tel->mm_conv.cell_idx = TELEMETER_PROFILE_ARRAY_LENGTH;
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

		ssd1306ClearScreen(MAIN_AREA);

		ssd1306DrawString(1,0, "   AVRG 1/10mm", &Font_5x8);

		ssd1306PrintInt(1, 9 , "FL ", (int32_t) getTelemeterAvrg(TELEMETER_FL), &Font_5x8);
		ssd1306PrintInt(1, 18, "DL ", (int32_t) getTelemeterAvrg(TELEMETER_DL), &Font_5x8);
		ssd1306PrintInt(1, 27, "DR ", (int32_t) getTelemeterAvrg(TELEMETER_DR), &Font_5x8);
		ssd1306PrintInt(1, 36, "FR ", (int32_t) getTelemeterAvrg(TELEMETER_FR), &Font_5x8);

		ssd1306PrintInt(45, 9 ,"", (int32_t) (telemeters.FL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 18,"", (int32_t) (telemeters.DL.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 27,"", (int32_t) (telemeters.DR.dist_mm * 10), &Font_5x8);
		ssd1306PrintInt(45, 36,"", (int32_t) (telemeters.FR.dist_mm * 10), &Font_5x8);

		//		ssd1306PrintInt(1, 45, "IT TIME  =  ", (int32_t) telemeters.it_time, &Font_5x8);
		ssd1306PrintInt(1, 54, "IT ERROR =  ", (telemeters.it_cnt - telemeters.end_of_conversion), &Font_5x8);
		ssd1306Refresh(MAIN_AREA);

		if (joy == JOY_RIGHT)
		{
			HAL_Delay(1000);
			while (joy != JOY_LEFT)
			{
				joy = expanderJoyFiltered();
				ssd1306ClearScreen(MAIN_AREA);
				ssd1306DrawString(1,0, "   ADC  REF  VAR", &Font_5x8);

				ssd1306PrintInt(1, 9 , "FL ", (int32_t) telemeters.FL.adc, &Font_5x8);
				ssd1306PrintInt(1, 18, "DL ", (int32_t) telemeters.DL.adc, &Font_5x8);
				ssd1306PrintInt(1, 27, "DR ", (int32_t) telemeters.DR.adc, &Font_5x8);
				ssd1306PrintInt(1, 36, "FR ", (int32_t) telemeters.FR.adc, &Font_5x8);

//				ssd1306PrintInt(45, 9 , "", (int32_t) getTelemeterAvrg(TELEMETER_FL), &Font_5x8);
//				ssd1306PrintInt(45, 18, "", (int32_t) getTelemeterAvrg(TELEMETER_DL), &Font_5x8);
//				ssd1306PrintInt(45, 27, "", (int32_t) getTelemeterAvrg(TELEMETER_DR), &Font_5x8);
//				ssd1306PrintInt(45, 36, "", (int32_t) getTelemeterAvrg(TELEMETER_FR), &Font_5x8);

				ssd1306PrintInt(75, 9 , "", (int32_t) getTelemeterSpeed(TELEMETER_FL), &Font_5x8);
				ssd1306PrintInt(75, 18, "", (int32_t) getTelemeterSpeed(TELEMETER_DL), &Font_5x8);
				ssd1306PrintInt(75, 27, "", (int32_t) getTelemeterSpeed(TELEMETER_DR), &Font_5x8);
				ssd1306PrintInt(75, 36, "", (int32_t) getTelemeterSpeed(TELEMETER_FR), &Font_5x8);

				ssd1306Refresh(MAIN_AREA);
			}
			while (joy == JOY_LEFT)
			{
				joy = expanderJoyFiltered();
			}
		}
	}
	telemetersStop();
}


void telemetersGetCalibrationValues(void)
{
    int i;

    bluetoothPrintf("Front Left; Front Right; Diagonal Left; Diagonal Right\n");
    for (i = 0; i < TELEMETER_PROFILE_ARRAY_LENGTH; i++)
    {
        bluetoothWaitReady();
        bluetoothPrintf("%d; %d; %d; %d\n",
                        telemeters_profile->front.left[i],
                        telemeters_profile->front.right[i],
                        telemeters_profile->diag.left[i],
                        telemeters_profile->diag.right[i]);

    }
}
