/**************************************************************************/
/*!
    @file     ADXRS620.h
    @author   PLF Pacabot.com
    @date     03 August 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __TELEMETERS_H__
#define __TELEMETERS_H__

/* Module Identifier */
#include "config/module_id.h"
#include "application/statistiques/statistiques.h"

/* Error codes */
#define TELEMETERS_DRIVER_E_SUCCESS  0
#define TELEMETERS_DRIVER_E_ERROR    MAKE_ERROR(TELEMETERS_DRIVER_MODULE_ID, 1)

#define TX_ON			1
#define TX_OFF			2

#define DISTANCE_MEASURED	200
#define NUMBER_OF_CELL 		100
#define NUMBER_OF_MILLIMETER_BY_LOOP DISTANCE_MEASURED/NUMBER_OF_CELL
#define MIN_TELEMETERS_SPEED 20

/* Definition for ADCx Channel Pin */
#define TX_FL				GPIO_PIN_1
#define TX_FR				GPIO_PIN_10
#define TX_DUAL_DIAG		GPIO_PIN_11

/* Definition for ADCx's Channel */
#define RX_FL				ADC_CHANNEL_6
#define RX_DL				ADC_CHANNEL_5
#define RX_DR				ADC_CHANNEL_10
#define RX_FR				ADC_CHANNEL_11

extern int telemeter_FR_profile[NUMBER_OF_CELL + 1];
extern int telemeter_FL_profile[NUMBER_OF_CELL + 1];
extern int telemeter_DR_profile[NUMBER_OF_CELL + 1];
extern int telemeter_DL_profile[NUMBER_OF_CELL + 1];

/* Types definitions */

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

extern telemetersStruct telemeters;

void   telemetersInit(void);
void   telemetersStart(void);
void   telemetersStop(void);
void   telemeters_IT(void);
void   telemeters_DMA_IT(void);
void   telemetersAdc2Start(void);
void   telemetersAdc3Start(void);
void   telemeters_ADC2_IT(void);
void   telemeters_ADC3_IT(void);
void   getTelemetersADC(telemeterStruct *tel, ADC_HandleTypeDef *hadc);
void   getTelemetersDistance (telemeterStruct *tel);
int	   getTelemetersVariation(telemeterStruct *tel);
void   telemetersTest(void);

#endif
