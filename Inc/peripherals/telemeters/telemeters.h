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

/* Definition for ADCx Channel Pin */
#define TX_FL				GPIO_PIN_1
#define TX_FR				GPIO_PIN_10
#define TX_DUAL_DIAG		GPIO_PIN_11

/* Definition for ADCx's Channel */
#define RX_FL				ADC_CHANNEL_6
#define RX_DL				ADC_CHANNEL_5
#define RX_DR				ADC_CHANNEL_10
#define RX_FR				ADC_CHANNEL_11

/* Types definitions */

/* Types definitions */
enum telemeterType {FL, DL, DR, FR};

typedef struct
{
    float dist_mm;
	int old_avrg;
	int cell_idx;
	int	*profile;
} telemeterConvStruct;

typedef struct
{
	telemeterConvStruct mm_conv;
    int adc;
    int adc_ref;
    mobileAvrgStruct mAvrgTypeDef;
    mobileAvrgStruct mAvrgTypeDef_ref;
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

	uint64_t it_cnt;
	uint64_t end_of_conversion;
	char active_state;
	char selector;
} telemetersStruct;

extern telemetersStruct telemeters;

void telemetersInit(void);
void telemetersStart(void);
void telemetersStop(void);
void telemeters_IT(void);
void telemeters_DMA_IT(void);
void telemetersAdc2Start(void);
void telemetersAdc3Start(void);
void telemeters_ADC2_IT(void);
void telemeters_ADC3_IT(void);
void setTelemetersADC(telemeterStruct *tel, ADC_HandleTypeDef *hadc);
float getTelemeterDist (enum telemeterType what_telemeter);
float getTelemetersDistance (telemeterStruct *tel);
void telemetersTest(void);

#endif
