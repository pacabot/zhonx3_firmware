/**************************************************************************/
/*!
    @file    line_follower.c
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
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/tone/tone.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"

/* Middleware declarations */
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/followControl.h"
#include "middleware/controls/motionControl/mainControl.h"

/* Declarations for this module */
#include "application/lineFollower/lineFollower.h"

line_follower_struct line_follower;

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

void LineTest(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	tone(a, 500);
	HAL_Delay(1000);
	double gauche_max=1000.0/(double)lineSensors.left.adc_value;
	double devant_max=1000.0/(double)lineSensors.front.adc_value;
	double droite_max=1000.0/(double)lineSensors.right.adc_value;
	tone(c, 500);

	follow_control.follow_type = FOLLOW_LINE;

	move(0, 10000, 50, 0);
	while(isEndMove() != TRUE);

	while(expanderJoyFiltered()!=JOY_LEFT)
	{
		line_follower_test(gauche_max,devant_max,droite_max);

		ssd1306ClearScreen();
		ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 54, "Error =  ", (int32_t) follow_control.follow_error, &Font_5x8);
		ssd1306Refresh();
	}
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

void line_follower_test(double CoefLeft, double CoefFront, double CoefRight)
{
	line_follower.position = 0.00;
	float gauche=(float)lineSensors.left.adc_value*CoefLeft;
	float devant=(float)lineSensors.front.adc_value*CoefFront;
	float droite=(float)lineSensors.right.adc_value*CoefRight;

	line_follower.position = (gauche-droite) * -0.01;
	if ((devant*1.2) < gauche || (devant*1.2) < droite)
	{
		line_follower.position = (gauche-droite) * -0.02;
	}

}
