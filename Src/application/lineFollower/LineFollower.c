/**************************************************************************/
/*!
    @file    LineFollower.c
    @author   BM Pacabot.com
    @date     05 May 2015
    @version  0.00
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

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
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/tone/tone.h"

/* Middleware declarations */
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "application/lineFollower/lineFollower.h"

lineFollower_struct lineFollower;

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

void LineTest(void)
{

	lineSensorsInit();
	lineSensorsStart();
	tone(a, 500);
	HAL_Delay(2000);
	double gauche_max=1000.0/(double)lineSensors.left.adc_value;
	double devant_max=1000.0/(double)lineSensors.front.adc_value;
	double droite_max=1000.0/(double)lineSensors.right.adc_value;
	tone(c, 500);
//	mainControlInit();
//	HAL_Delay(500);

//	move(0, 0, 500, 400);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		lineFollower_test(gauche_max,devant_max,droite_max);

		ssd1306ClearScreen();
		ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);
		ssd1306Refresh();
	}

}

void lineFollower_test(double CoefLeft, double CoefFront, double CoefRight)
{
	lineFollower.position = 0.00;
	int gauche=lineSensors.left.adc_value*CoefLeft;
	int devant=lineSensors.front.adc_value*CoefFront;
	int droite=lineSensors.right.adc_value*CoefRight;

	lineFollower.position=(gauche-droite)/100;
	if ((devant*1.2)<gauche || (devant*1.2)<droite)
	{
		lineFollower.position=(gauche-droite)/10;
	}
}
