/**************************************************************************/
/*!
    @file     lineSensors.h
    @author   PLF Pacabot.com
    @date     01 December 2014
    @version  0.10
*/
/**************************************************************************/
#ifndef __LINESENSORS_H__
#define __LINESENSORS_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define LINESENSORS_DRIVER_E_SUCCESS  0
#define LINESENSORS_DRIVER_E_ERROR    MAKE_ERROR(LINESENSORS_DRIVER_MODULE_ID, 1)

/* Definition for ADCx GPIO Pin */
#define TX_LINESENSORS				GPIO_PIN_2		//PA2

/* Definition for ADCx's Channel */
#define RX_LEFT_EXT					ADC_CHANNEL_3	//ADC3
#define RX_LEFT						ADC_CHANNEL_4  	//ADC2
#define RX_FRONT					ADC_CHANNEL_1	//ADC3
#define RX_RIGHT					ADC_CHANNEL_13 	//ADC2
#define RX_RIGHT_EXT				ADC_CHANNEL_12	//ADC3


/* Types definitions */
typedef struct
{
	uint16_t average_value;
    uint16_t adc_value;
    uint16_t offset;
    uint16_t higher_interval;
    uint16_t lower_interval;
    char line_presence;
}lineSensors_state;

typedef struct
{
	lineSensors_state left_ext;
	lineSensors_state left;
	lineSensors_state front;
	lineSensors_state right;
	lineSensors_state right_ext;
	char active_ADC2;
	char active_ADC3;
	char emitter_state;
	char active_state;
} lineSensors_struct;

volatile lineSensors_struct lineSensors;

void lineSensorsInit(void);
void lineSensorsStart(void);
void lineSensorsStop(void);
void lineSensorsStop_DMA_ADC1(void);
void lineSensorsStop_DMA_ADC3(void);
void lineSensorsCalibrate(void);
void lineSensors_IT(void);
void lineSensors_ADC_IT(ADC_HandleTypeDef *hadc);
void lineSensorsTest(void);

#endif
