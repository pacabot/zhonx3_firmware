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


/* Definition for ADCx GPIO Pin */
#define TX_LINESENSORS				GPIO_PIN_2		//PA2

/* Definition for ADCx's Channel */
#define RX_LEFT_EXT					ADC_CHANNEL_3	//ADC3
#define RX_LEFT						ADC_CHANNEL_4  	//ADC1
#define RX_FRONT					ADC_CHANNEL_1	//ADC3
#define RX_RIGHT					ADC_CHANNEL_13 	//ADC1
#define RX_RIGHT_EXT				ADC_CHANNEL_12	//ADC3

/* Types definitions */
typedef struct
{
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
	char active_ADC1;
	char active_ADC3;
	char active_state;
} lineSensors_struct;

volatile lineSensors_struct lineSensors;

void lineSensorsInit(void);
void lineSensorsStart(void);
void lineSensorsStop_DMA_ADC1(void);
void lineSensorsStop_DMA_ADC3(void);
void lineSensorsCalibrate(void);
void lineSensors_IT(void);

#endif
