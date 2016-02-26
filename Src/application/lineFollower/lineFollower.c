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
int _Factor=10;

//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;


void lineSensorSendBluetooth(void)
{
	ground_sensors_struct current;

	int i=0;
	int GO=0;

	mainControlInit();
	telemetersStop();
	lineSensorsInit();
	lineSensorsStart();
	motorsInit();
	motorsDriverSleep(OFF);


	tone(e, 500);
	move(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE){}
	move(-80, 0, 25, 0);
	max_Floor.left=lineSensors.left.adc_value;
	max_Floor.front=lineSensors.front.adc_value;
	max_Floor.right=lineSensors.right.adc_value;
	max_Floor.leftExt=lineSensors.left_ext.adc_value;
	max_Floor.rightExt=lineSensors.right_ext.adc_value;
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	memcpy(&current, &min_Floor, sizeof(ground_sensors_struct) );

	while(hasMoveEnded() != TRUE)
	{
		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		if (current.left < min_Floor.left) min_Floor.left = current.left;
		if (current.front < min_Floor.front) min_Floor.front = current.front;
		if (current.right < min_Floor.right) min_Floor.right = current.right;
		if (current.leftExt < min_Floor.leftExt) min_Floor.leftExt = current.leftExt;
		if (current.rightExt < min_Floor.rightExt) min_Floor.rightExt = current.rightExt;

		if (current.left > max_Floor.left) max_Floor.left = current.left;
		if (current.front > max_Floor.front) max_Floor.front = current.front;
		if (current.right > max_Floor.right) max_Floor.right = current.right;
		if (current.leftExt > max_Floor.leftExt) max_Floor.leftExt = current.leftExt;
		if (current.rightExt > max_Floor.rightExt) max_Floor.rightExt = current.rightExt;

		i++;
		if (current.leftExt>min_Floor.leftExt*1.5) GO=i;
		if(i%10==0 && GO)
		{

// ===================================================================================
// Envoi BLUETOOTH
// ===================================================================================

			bluetoothPrintf("%d , %d , %d , %d , %d \n",current.leftExt,current.left,current.front,current.right,current.rightExt);
	/*		ssd1306ClearScreen(MAIN_AREA);
			ssd1306DrawString(10, 5, "send hello ZHONX III", &Font_5x8);
			ssd1306PrintInt(10, 15, "",current.leftExt, &Font_5x8);
			ssd1306PrintInt(10, 25, ",",current.left, &Font_5x8);
			ssd1306PrintInt(10, 35, ",",current.front, &Font_5x8);
			ssd1306PrintInt(10, 45, ",",current.right, &Font_5x8);
			ssd1306PrintInt(10, 55, ",",current.rightExt, &Font_5x8);
			ssd1306Refresh(MAIN_AREA); */
		}
	}
	move(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE){}
	// desactivate PID
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);
}

//----------------------------------------------------------------
// Initialize data sensor to memorize the max and min value for each 5 sensors
void lineSensorsCalibration(void)
{
	mainControlInit();
	lineSensorsInit();

	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(NO_FOLLOW);

	HAL_Delay(1000);

	double cdg=0;
	double cdg2=0;
	double A,B,C,D,E;
	ground_sensors_struct current;

	lineSensorsStart();

	tone(e, 500);
	motorsDriverSleep(OFF);
	move(40, 0, 100, 100);
	while(hasMoveEnded() != TRUE);
	move(-80, 0, 150, 150);
// -------------------------------------------------------------
// Init line Sensor

	max_Floor.left=lineSensors.left.adc_value;
	max_Floor.front=lineSensors.front.adc_value;
	max_Floor.right=lineSensors.right.adc_value;
	max_Floor.leftExt=lineSensors.left_ext.adc_value;
	max_Floor.rightExt=lineSensors.right_ext.adc_value;
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	memcpy(&current, &min_Floor, sizeof(ground_sensors_struct) );

	while(hasMoveEnded() != TRUE)
	{

		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		if (current.left < min_Floor.left) min_Floor.left = current.left;
		if (current.front < min_Floor.front) min_Floor.front = current.front;
		if (current.right < min_Floor.right) min_Floor.right = current.right;
		if (current.leftExt < min_Floor.leftExt) min_Floor.leftExt = current.leftExt;
		if (current.rightExt < min_Floor.rightExt) min_Floor.rightExt = current.rightExt;

		if (current.left > max_Floor.left) max_Floor.left = current.left;
		if (current.front > max_Floor.front) max_Floor.front = current.front;
		if (current.right > max_Floor.right) max_Floor.right = current.right;
		if (current.leftExt > max_Floor.leftExt) max_Floor.leftExt = current.leftExt;
		if (current.rightExt > max_Floor.rightExt) max_Floor.rightExt = current.rightExt;

	}
	tone(b, 500);
	tone(c, 500);

	move(40, 0, 30, 30);

	ssd1306ClearScreen(MAIN_AREA);
	while(hasMoveEnded() != TRUE)
	{  //=($A$1*A3+$B$1*B3+$C$1*C3+$D$1*D3+$E$1*E3)/(A3+B3+C3+D3+E3)

		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
		B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
		C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
		D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
		E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

		ssd1306ClearScreen(MAIN_AREA);
   	    ssd1306DrawLine(0,32, 127, 32);
		cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
		cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);



		ssd1306PrintInt(10, 40,  "Centre= ", (uint16_t)cdg, &Font_5x8);
		ssd1306PrintInt(10, 50,  "milieu= ", (uint16_t)D-B, &Font_5x8);
		ssd1306Refresh(MAIN_AREA);
	}

	// desactivate PID
	pid_loop.start_state = FALSE;
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);

	int joystick = expanderJoyFiltered();
	while (joystick!=JOY_LEFT)
	{

	joystick = expanderJoyFiltered();

	current.left=lineSensors.left.adc_value;
	current.front=lineSensors.front.adc_value;
	current.right=lineSensors.right.adc_value;
	current.leftExt=lineSensors.left_ext.adc_value;
	current.rightExt=lineSensors.right_ext.adc_value;

	A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
	B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
	C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
	D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
	E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

	ssd1306ClearScreen(MAIN_AREA);
	    ssd1306DrawLine(0,32, 127, 32);
	cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
	cdg=(-1000*A-389*B+D*431+E*1000)/(A+B+C+D+E);

	cdg2= cdg*C/500;
	cdg=D-B;
	if (cdg<0)
	{
		cdg2=C-1000;
	} else
	{
		cdg2=1000-C;
	}
	cdg=D-B;
	if (cdg<0)
	{
		cdg2=-B;
	} else
	{
		cdg2=D;
	}
	line_follower.position = (cdg2)/1000.0;

	if (cdg>0)
	{
		ssd1306PrintInt(10, 40,  "Centre= ", (uint16_t)cdg, &Font_5x8);
	} else
	{
		ssd1306PrintInt(10, 40,  "Centre=-", (uint16_t)-cdg, &Font_5x8);
	}
	if (D>B)
	{
	    ssd1306PrintInt(10, 50,  "milieu= ", (uint16_t)cdg2, &Font_5x8);
	} else
	{
		ssd1306PrintInt(10, 50,  "milieu=-", (uint16_t)-cdg2, &Font_5x8);
	}
	ssd1306Refresh(MAIN_AREA);

	}
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{
	ground_sensors_struct current;
	double A,B,C,D,E;
	double cdg=0;
	double cdg2=0;

	mainControlInit();
	lineSensorsInit();

	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(LINE_FOLLOW);

	lineSensorsStart();
	motorsDriverSleep(OFF);

	if (max_Floor.left-min_Floor.left< 100.0)
	{
		tone(a, 500);
		tone(b, 500);
	    return;
	}

	tone(c, 100);
	coef_Floor.left=100.0/(max_Floor.left-min_Floor.left);     //  100/(max_capteur-min_capteur) (0..100)
	coef_Floor.front=100.0/(max_Floor.front-min_Floor.front);
	coef_Floor.right=100.0/(max_Floor.right-min_Floor.right);
	coef_Floor.leftExt=100.0/(max_Floor.leftExt-min_Floor.leftExt);
	coef_Floor.rightExt=100.0/(max_Floor.rightExt-min_Floor.rightExt);



	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) min_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) min_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) min_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) min_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) min_Floor.rightExt, &Font_5x8);
	ssd1306Refresh(MAIN_AREA);
	HAL_Delay(1000);
	tone(c, 100);

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintInt(10, 5,  "LEFT_EXT  =  ", (uint16_t) max_Floor.leftExt, &Font_5x8);
	ssd1306PrintInt(10, 15, "LEFT      =  ", (uint16_t) max_Floor.left, &Font_5x8);
	ssd1306PrintInt(10, 25, "FRONT --  =  ", (uint16_t) max_Floor.front, &Font_5x8);
	ssd1306PrintInt(10, 35, "RIGHT     =  ", (uint16_t) max_Floor.right, &Font_5x8);
	ssd1306PrintInt(10, 45, "RIGHT_EXT =  ", (uint16_t) max_Floor.rightExt, &Font_5x8);
	ssd1306Refresh(MAIN_AREA);
	HAL_Delay(1000);


//	HAL_Delay(500);

	line_follower.active_state = TRUE;
	move(0, 10000, MAXSPEED, MAXSPEED);
//	while(hasMoveEnded() != TRUE);
	char foreward = TRUE;
	char cpt=0;
	int  error;


	while(expanderJoyFiltered()!=JOY_LEFT && foreward)
	{


		current.left=lineSensors.left.adc_value;
		current.front=lineSensors.front.adc_value;
		current.right=lineSensors.right.adc_value;
		current.leftExt=lineSensors.left_ext.adc_value;
		current.rightExt=lineSensors.right_ext.adc_value;

		A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
		B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
		C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
		D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
		E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

		ssd1306ClearScreen(MAIN_AREA);
		    ssd1306DrawLine(0,32, 127, 32);
		cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
		cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);

		cdg=D-B;
		if (cdg<0)
		{
			cdg2=-B;
			//if (cdg<-500) move(0, 10000, 500, 0);
		} else
		{
			cdg2=D;
			//if (cdg>500)move(0, 10000, 500, 0);
		}

		//error=follow_control.follow_error*10;

		error=(int) cdg2/1000.0;


		ssd1306ClearScreen(MAIN_AREA);
		ssd1306PrintInt(5, 5,  "LEFT_EXT  =  ", current.leftExt, &Font_5x8);
		ssd1306PrintInt(5, 15, "LEFT      =  ", current.left, &Font_5x8);
		ssd1306PrintInt(5, 25, "FRONT --  =  ", current.front, &Font_5x8);
		ssd1306PrintInt(5, 35, "RIGHT     =  ", current.right, &Font_5x8);
		ssd1306PrintInt(5, 45, "RIGHT_EXT =  ", current.rightExt, &Font_5x8);


		ssd1306PrintInt(70, 55, "error=", error, &Font_5x8);
		ssd1306Refresh(MAIN_AREA);


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

int EstAGauche()
{
	ground_sensors_struct current;
	current.left=lineSensors.left.adc_value;
	current.front=lineSensors.front.adc_value;
	current.right=lineSensors.right.adc_value;
	current.leftExt=lineSensors.left_ext.adc_value;
	current.rightExt=lineSensors.right_ext.adc_value;
	if ( (current.left>current.right)||
	     (current.leftExt>max_Floor.leftExt/2)) return 1;
}

//----------------------------------------------------------------------
// fonction pour asservir zhonx sur la ligne
//
void controlLoop(void)
{
	ground_sensors_struct current;
	double A,B,C,D,E;
	double cdg=0;
	double cdg2=0.0;

	current.left=lineSensors.left.adc_value;
	current.front=lineSensors.front.adc_value;
	current.right=lineSensors.right.adc_value;
	current.leftExt=lineSensors.left_ext.adc_value;
	current.rightExt=lineSensors.right_ext.adc_value;

//	A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
	B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
	C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
	D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
//	E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;


//	cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
//  cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);
	cdg=D-B;
	if (cdg<0)
	{
		cdg2=-B;
	} else
	{
		cdg2=D;
	}
	line_follower.position = (cdg2)/1000.0;

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
//	if (hasMoveEnded() == TRUE)
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

