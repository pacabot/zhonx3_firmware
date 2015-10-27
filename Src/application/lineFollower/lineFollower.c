/**************************************************************************/
/*!
    @file    line_follower.c
    @author   BM Pacabot.com
    @date     05 May 2015
    @version  0.01
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
#include "middleware/controls/motionControl/mainControl.h"
#include "middleware/controls/motionControl/wallFollowControl.h"

/* Declarations for this module */
#include "application/lineFollower/lineFollower.h"
#include "application/statistiques/statistiques.h"

line_follower_struct line_follower;
ground_sensors_struct max_Floor;	//global data to memorize maximum value of sensors
ground_sensors_struct coef_Floor;	//global data to memorize coeff value (0..1000]
ground_sensors_struct min_Floor;	//global data to memorize minimum value of sensors

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;

//----------------------------------------------------------------
// Initialize data sensor to memorize the max and min value for each 5 sensors
void lineSensorsCalibration(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	motorsDriverSleep(OFF);

	tone(a, 500);
//	HAL_Delay(1000);
	move(0, 100, 200, 0);

// -------------------------------------------------------------
// Init line Sensor

	max_Floor.left=(double)lineSensors.left.adc_value;
	max_Floor.front=(double)lineSensors.front.adc_value;
	max_Floor.right=(double)lineSensors.right.adc_value;
	max_Floor.leftExt=(double)lineSensors.left_ext.adc_value;
	max_Floor.rightExt=(double)lineSensors.right_ext.adc_value;
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	while(hasMoveEnded() != TRUE)
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
	tone(c, 500);

	// desactivate PID
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{
	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	control_params.line_follow_state = TRUE;
	motorsDriverSleep(OFF);


	if (max_Floor.left-min_Floor.left< 100.0)
	{
		tone(a, 3000);
		max_Floor.left=2000.0;
		max_Floor.front=2000.0;
		max_Floor.right=2000.0;
		max_Floor.leftExt=2000.0;
		max_Floor.rightExt=2000.0;
		min_Floor.left=150.0;
		min_Floor.front=150.0;
		min_Floor.right=150.0;
		min_Floor.leftExt=150.0;
		min_Floor.rightExt=150.0;
		tone(b, 3000);

	//	return;
	}

	tone(c, 100);
	coef_Floor.left=1000.0/(max_Floor.left-min_Floor.left);     //  1000/(max_capteur-min_capteur)
	coef_Floor.front=1000.0/(max_Floor.front-min_Floor.front);
	coef_Floor.right=1000.0/(max_Floor.right-min_Floor.right);
	coef_Floor.leftExt=1000.0/(max_Floor.leftExt-min_Floor.leftExt);
	coef_Floor.rightExt=1000.0/(max_Floor.rightExt-min_Floor.rightExt);



	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) min_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) min_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) min_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) min_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) min_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
//	HAL_Delay(900);
	tone(c, 100);

	ssd1306ClearScreen();
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) max_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) max_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) max_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) max_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) max_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
//	HAL_Delay(900);


//	HAL_Delay(500);

	line_follower.active_state = TRUE;
	move(0, 10000, MAXSPEED, 0);
//	while(isEndMove() != TRUE);
	char foreward = TRUE;
	char cpt=0;
	int  error;
	while(expanderJoyFiltered()!=JOY_LEFT && foreward)
	{
		//error=follow_control.follow_error*10;
		int left=((double)lineSensors.left.adc_value - min_Floor.left) * coef_Floor.left ;
		int front=((double)lineSensors.front.adc_value- min_Floor.front) * coef_Floor.front ;
		int right=((double)lineSensors.right.adc_value- min_Floor.right) * coef_Floor.right ;
		error=line_follower.position*200;
		ssd1306ClearScreen();
//		ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) lineSensors.left_ext.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) lineSensors.left.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) lineSensors.front.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) lineSensors.right.adc_value, &Font_5x8);
//		ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) lineSensors.right_ext.adc_value, &Font_5x8);

		ssd1306PrintInt(10, 15, "LEFT      =  ", left, &Font_5x8);
		ssd1306PrintInt(10, 25, "FRONT --  =  ", front, &Font_5x8);
		ssd1306PrintInt(10, 35, "RIGHT     =  ", right, &Font_5x8);

		ssd1306PrintInt(10, 54, "Error =  ", error, &Font_5x8);
		ssd1306Refresh();


// -----------------------------------------------------------------------
// Condition to stop zhonx if no line
// -----------------------------------------------------------------------
		if ((double)lineSensors.front.adc_value < min_Floor.front *1.2 &&
			(double)lineSensors.left.adc_value < min_Floor.left *1.2 &&
			(double)lineSensors.right.adc_value < min_Floor.right *1.2 &&
			(double)lineSensors.left_ext.adc_value < min_Floor.leftExt *1.2 &&
			(double)lineSensors.right_ext.adc_value < min_Floor.rightExt *1.2)
		{
			cpt++;
			if (cpt>5)
			{
			    foreward = FALSE;
			    move(0, 150, 250, 0);
			    tone(c, 500);tone(d, 500);
			}
		}
// -----------------------------------------------------------------------
// Condition to stop if right priority
// -----------------------------------------------------------------------
//		if (((double)lineSensors.left_ext.adc_value*1.2) > max_Floor.leftExt )
//		{
//			move(0, 30, 30, 0);
//			tone(c, 500);
//			// capteur telemeter ON
//			move(0, 10000, MAXSPEED, 0);
//		}

	}
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);
}

//----------------------------------------------------------------------
// fonction pour asservir zhonx sur la ligne
//
void controlLoop(void)
{
	static int maxfront=0;  // memorize the max level of front sensors line

	int left=(lineSensors.left.adc_value - min_Floor.left) * coef_Floor.left ;
	int front=(lineSensors.front.adc_value- min_Floor.front) * coef_Floor.front ;
	int right=(lineSensors.right.adc_value- min_Floor.right) * coef_Floor.right ;
    int rightExt=(lineSensors.right_ext.adc_value - min_Floor.rightExt) * coef_Floor.rightExt;
    int leftExt=(lineSensors.left_ext.adc_value  - min_Floor.leftExt)* coef_Floor.leftExt;

    int midle=0;		// take account if the center sensor line is out the line
    int inside=right-left; //take account the sensor just right and left of front
    int	outside=0;	// take account the external sensor line


	if (inside>20)
	{
		midle=(maxfront-front);
	}
	else if (inside<-20)
	{
		midle=-(maxfront-front);
	}else
	{
		maxfront=front;
	}
	// check if we are for the center out of the line to take account the gaucheExt and droiteExt
//    if (devant<100)
//    {
//    	exterieur = droiteExt - gaucheExt;
//    }

    line_follower.position = (double)(right - left + midle + outside) * 0.004;

}
void lineFollower_IT(void)
{
	// Rapide
//	static int vitesse=0;

	controlLoop();

//	if (follow_control.follow_error > 3.0 && vitesse==0)
//	{
//		// deceleration
//		move(0, 30, MAXSPEED, MINSPEED);
//		vitesse=-1;
//	}
//	else if (follow_control.follow_error < 3.0 && vitesse==0)
//	{
//		// acceleration
//		move(0, 30, MINSPEED, MAXSPEED);
//		vitesse=1;
//	}
//
//	if (isEndMove() == TRUE)
//	{
//		if (vitesse<0)
//		{
//			move(0, 10000, MINSPEED, MINSPEED);
//		}
//		else if (vitesse>0)
//		{
//			move(0, 10000, MAXSPEED, MAXSPEED);
//		}
//		vitesse=0;
//	}
	// -----------------------------------------------------------------------
	// Condition to stop zhonx if no line
	// -----------------------------------------------------------------------
	if ((double)lineSensors.front.adc_value < min_Floor.front *1.2 &&
		(double)lineSensors.left.adc_value < min_Floor.left *1.2 &&
		(double)lineSensors.right.adc_value < min_Floor.right *1.2 &&
		(double)lineSensors.left_ext.adc_value < min_Floor.leftExt *1.2 &&
		(double)lineSensors.right_ext.adc_value < min_Floor.rightExt *1.2)
	{
	    move(0, 150, 250, 0);
	}
}

