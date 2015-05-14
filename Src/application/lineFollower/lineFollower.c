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
#include "application/statistiques/statistiques.h"

line_follower_struct line_follower;
ground_sensors_struct max_Floor;
ground_sensors_struct coef_Floor;
ground_sensors_struct min_Floor;

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

void lineTest(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();

	tone(a, 500);
	HAL_Delay(1000);
	move(0, 100, 50, 0);

// -------------------------------------------------------------
// Init line Sensor

	max_Floor.left=(double)lineSensors.left.adc_value;
	max_Floor.front=(double)lineSensors.front.adc_value;
	max_Floor.right=(double)lineSensors.right.adc_value;
	max_Floor.leftExt=(double)lineSensors.left_ext.adc_value;
	max_Floor.rightExt=(double)lineSensors.right_ext.adc_value;
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	while(isEndMove() != TRUE)
	{
		if (lineSensors.left.adc_value < min_Floor.left) min_Floor.left = lineSensors.left.adc_value;
		if (lineSensors.front.adc_value < min_Floor.front) min_Floor.front = lineSensors.front.adc_value;
		if (lineSensors.right.adc_value < min_Floor.right) min_Floor.right = lineSensors.right.adc_value;
		if (lineSensors.left_ext.adc_value < min_Floor.leftExt) min_Floor.leftExt = lineSensors.left_ext.adc_value;
		if (lineSensors.right_ext.adc_value < min_Floor.rightExt) min_Floor.rightExt = lineSensors.right_ext.adc_value;

		if (lineSensors.left.adc_value > max_Floor.left) max_Floor.left = lineSensors.left.adc_value;
		if (lineSensors.front.adc_value > max_Floor.front) max_Floor.front = lineSensors.front.adc_value;
		if (lineSensors.right.adc_value > max_Floor.right) max_Floor.right = lineSensors.right.adc_value;
		if (lineSensors.left_ext.adc_value > max_Floor.leftExt) max_Floor.leftExt = lineSensors.left_ext.adc_value;
		if (lineSensors.right_ext.adc_value > max_Floor.rightExt) max_Floor.rightExt = lineSensors.right_ext.adc_value;
	}

	tone(b, 500);
	HAL_Delay(2000);
	tone(c, 500);

	tone(d, 500);
	coef_Floor.left=1000.0/(max_Floor.left-min_Floor.left);     //  1000/(max_capteur-min_capteur)
	coef_Floor.front=1000.0/(max_Floor.front-min_Floor.front);
	coef_Floor.right=1000.0/(max_Floor.right-min_Floor.right);
	coef_Floor.leftExt=1000.0/(max_Floor.leftExt-min_Floor.leftExt);
	coef_Floor.rightExt=1000.0/(max_Floor.rightExt-min_Floor.rightExt);

	HAL_Delay(100);

	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) min_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) min_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) min_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) min_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) min_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
	HAL_Delay(100);

	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) max_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) max_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) max_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) max_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) max_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
	HAL_Delay(2000);


	follow_control.follow_type = FOLLOW_LINE;

	line_follower.active_state = TRUE;
	move(0, 10000, 1000, 1000);
//	while(isEndMove() != TRUE);
	char marche = TRUE;
	char cpt=0;

	while(expanderJoyFiltered()!=JOY_LEFT && marche)
	{
		ssd1306ClearScreen();
		ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);
		ssd1306PrintInt(10, 54, "Error =  ", (int32_t) follow_control.follow_error, &Font_5x8);
		ssd1306Refresh();


// -----------------------------------------------------------------------
// Condition to stop zhonx if no line
// -----------------------------------------------------------------------
		if ((double)lineSensors.front.adc_value < min_Floor.front *1.2 &&
			(double)lineSensors.left.adc_value < min_Floor.left *1.2 &&
			(double)lineSensors.right.adc_value < min_Floor.right *1.2)
		{
			cpt++;
			if (cpt>5)
			{
			    marche = FALSE;
			    move(0, 100, 0, 0);
			}
		} else
		{
			if (lineSensors.front.adc_value*1.3 > max_Floor.front)
			{
				cpt=0;
			}
		}
// -----------------------------------------------------------------------
// Condition to stop if right priority
// -----------------------------------------------------------------------
		if (((double)lineSensors.left_ext.adc_value*1.2) > max_Floor.leftExt )
		{
			move(0, 30, 30, 0);
			tone(c, 500);
			// capteur telemeter ON
			move(0, 10000, 250, 0);
		}

	}
	pid_loop.start_state = FALSE;
	telemetersStop();
	motorsSleepDriver(ON);
}

// fonction pour asservir zhonx sur la ligne
// -1 : ralentir
//  0 : meme vitesse
int asservissement(void)
{
	line_follower.position = 0.00;
	double gauche=(double)lineSensors.left.adc_value * coef_Floor.left - min_Floor.left;
	double devant=(double)lineSensors.front.adc_value * coef_Floor.front - min_Floor.front;
	double droite=(double)lineSensors.right.adc_value * coef_Floor.right - min_Floor.right;
//    double droiteExt=(double)lineSensors.right_ext.adc_value * coef_Floor.rightExt - min_Floor.rightExt;
//    double gaucheExt=(double)lineSensors.left_ext.adc_value * coef_Floor.leftExt - min_Floor.leftExt;

	line_follower.position = (droite-gauche) * 0.005;

	if ((devant*1.2) < gauche || (devant*1.2) < droite)
	{
		line_follower.position = (droite-gauche) * 0.01;
		return 1;
	}
	return 0;
}
void lineFollower_IT(void)
{
	// Rapide
	if asservissement();

	line_follower.position = 0.00;
	double gauche=(double)lineSensors.left.adc_value * coef_Floor.left - min_Floor.left;
	double devant=(double)lineSensors.front.adc_value * coef_Floor.front - min_Floor.front;
	double droite=(double)lineSensors.right.adc_value * coef_Floor.right - min_Floor.right;
//    double droiteExt=(double)lineSensors.right_ext.adc_value * coef_Floor.rightExt - min_Floor.rightExt;
//    double gaucheExt=(double)lineSensors.left_ext.adc_value * coef_Floor.leftExt - min_Floor.leftExt;


	line_follower.position = (droite-gauche) * 0.005;

	if ((devant*1.2) < gauche || (devant*1.2) < droite)
	{
		line_follower.position = (droite-gauche) * 0.01;
		move(0, 50, 1000, 250);
		while(isEndMove() != TRUE)
		{
			assert
		}
		move(0, 10000, 250, 250);

		{
			assert
		}
	}
}

