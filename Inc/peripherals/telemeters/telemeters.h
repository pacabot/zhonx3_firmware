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


/* Definition for ADCx Channel Pin */
#define TX_LEFT_FRONT				GPIO_PIN_1
#define TX_RIGHT_FRONT				GPIO_PIN_10
#define TX_DUAL_DIAG				GPIO_PIN_11

/* Definition for ADCx's Channel */
#define RX_LEFT_FRONT				ADC_CHANNEL_6
#define RX_LEFT_DIAG				ADC_CHANNEL_5
#define RX_RIGHT_DIAG				ADC_CHANNEL_10
#define RX_RIGHT_FRONT				ADC_CHANNEL_11

/* Types definitions */
typedef struct
{
    int16_t adc_value;
    int16_t telemeter_value;
    int64_t offset;
    uint16_t higher_interval;
    uint16_t lower_interval;
    char wall_presence;
    char emitter_state;
}telemeter_state;

typedef struct
{
	telemeter_state right_front;
	telemeter_state right_diag;
	telemeter_state left_front;
	telemeter_state left_diag;

	uint32_t it_cnt;
	uint32_t end_of_conversion;
	char active_state;
	char emitter_state;
	char selector;
} telemeters_struct;

volatile telemeters_struct telemeters;

void Telemeters_Init(void);
void Telemeters_Start(void);
void Telemeters_Stop(void);
void Telemeters_Calibrate(void);
void Telemeters_IT(void);
void Telemeters_DMA_IT(void);
void Telemeters_REGULAR_ADC_IT(void);
void Telemeters_INJECTED_ADC_IT(void);
void Debug_Telemeter(void);

#endif
