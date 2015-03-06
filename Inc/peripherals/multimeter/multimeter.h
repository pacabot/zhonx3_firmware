/**************************************************************************/
/*!
    @file     multimeter.h
    @author   PLF Pacabot.com
    @date     01 January 2015
    @version  0.10
*/
/**************************************************************************/
#ifndef __MULTIMETER_H__
#define __MULTIMETER_H__

/* Module Identifier */
#define MULTIMETER_DRIVER_MODULE_ID  10

/* Error codes */
#define MULTIMETER_DRIVER_E_SUCCESS  0
#define MULTIMETER_DRIVER_E_ERROR    MAKE_ERROR(MULTIMETER_DRIVER_MODULE_ID, 1)

/* Definition for ADCx Channel Pin */
#define GET_ADC_BAT			GPIO_PIN_0

/* Types definitions */
typedef struct
{
    uint16_t value;
    uint16_t offset;
    uint16_t higher_interval;
    uint16_t lower_interval;
    char under_value;
    char over_value;
}channel_meter;

typedef struct
{
	channel_meter vbat;
	channel_meter gyro_temp;
	channel_meter stm32_temp;
	uint32_t timer_cnt;
	char active_state;
	char get_vbat_state;
} multimeter_struct;

volatile multimeter_struct multimeter;

void mulimeterInit(void);
void multimeter_IT(void);
void multimeter_ADC_IT(void);
void mulimeterTest(void);

#endif
