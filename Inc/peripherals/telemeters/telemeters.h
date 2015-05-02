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

/* Error codes */
#define TELEMETERS_DRIVER_E_SUCCESS  0
#define TELEMETERS_DRIVER_E_ERROR    MAKE_ERROR(TELEMETERS_DRIVER_MODULE_ID, 1)

/* Definition for ADCx Channel Pin */
#define TX_LEFT_FRONT				GPIO_PIN_1
#define TX_RIGHT_FRONT				GPIO_PIN_10
#define TX_DUAL_DIAG				GPIO_PIN_11

/* Definition for ADCx's Channel */
#define RX_LEFT_FRONT				ADC_CHANNEL_6
#define RX_LEFT_DIAG				ADC_CHANNEL_5
#define RX_RIGHT_DIAG				ADC_CHANNEL_10
#define RX_RIGHT_FRONT				ADC_CHANNEL_11

#define SIZE_OF_AVEVAGE_TABLE 3

/* Types definitions */
typedef struct
{
    int16_t adc_value;
    int16_t telemeter_values[SIZE_OF_AVEVAGE_TABLE];
    int16_t telemeter_average;
    int64_t offset;
    uint16_t higher_interval;
    uint16_t lower_interval;
    char wall_presence;
    char sensor_state;
}telemeter_state;

typedef struct
{
	telemeter_state right_front;
	telemeter_state ref_right_front;
	telemeter_state right_diag;
	telemeter_state ref_right_diag;
	telemeter_state left_front;
	telemeter_state ref_left_front;
	telemeter_state left_diag;
	telemeter_state ref_left_diag;

	uint32_t it_cnt;
	uint32_t end_of_conversion;
	char active_state;
	char emitter_state;
	char selector;
	char ref_selector;
} telemeters_struct;

volatile telemeters_struct telemeters;

void telemetersInit(void);
void telemetersStart(void);
void telemetersStop(void);
void telemetersCalibrate(void);
void telemeters_IT(void);
void telemeters_DMA_IT(void);
void telemeters_ADC_IT(void);
void telemetersTest(void);

#endif
