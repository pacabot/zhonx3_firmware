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

/* Application declarations */
#include "application/statistiques/statistiques.h"

/* Middleware declarations */
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/moves/basicMoves/basicMoves.h"

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

/* Declarations for this module */
#include "application/lineFollower/lineFollower.h"

line_follower_struct line_follower;
ground_sensors_struct max_Floor;	//global data to memorize maximum value of sensors
ground_sensors_struct coef_Floor;	//global data to memorize coeff value (0..1000]
ground_sensors_struct min_Floor;	//global data to memorize minimum value of sensors
calibrate_sensors_struct sensor;
int _Factor=10;
int _Action=0;


//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;


void lineSensorSendBluetooth(void)
{
	ground_sensors_struct current;

	int i=0;
	int GO=0;

	lineSensorsInit();

	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(NO_FOLLOW);

	lineSensorsStart();

	tone(e, 500);
	basicMove(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE);
	basicMove(-80, 0, 25, 0);
	max_Floor.left = getLineSensorAdc(LINESENSOR_L);
	max_Floor.front = getLineSensorAdc(LINESENSOR_F);
	max_Floor.right = getLineSensorAdc(LINESENSOR_R);
	max_Floor.leftExt = getLineSensorAdc(LINESENSOR_EXT_L);
	max_Floor.rightExt = getLineSensorAdc(LINESENSOR_EXT_R);
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	memcpy(&current, &min_Floor, sizeof(ground_sensors_struct) );

	while(hasMoveEnded() != TRUE)
	{
		current.left = getLineSensorAdc(LINESENSOR_L);
		current.front = getLineSensorAdc(LINESENSOR_F);
		current.right = getLineSensorAdc(LINESENSOR_R);
		current.leftExt = getLineSensorAdc(LINESENSOR_EXT_L);
		current.rightExt = getLineSensorAdc(LINESENSOR_EXT_R);

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
			ssd1306PrintIntAtLine(10, 1, "",current.leftExt, &Font_5x8);
			ssd1306PrintIntAtLine(10, 2, ",",current.left, &Font_5x8);
			ssd1306PrintIntAtLine(10, 3, ",",current.front, &Font_5x8);
			ssd1306PrintIntAtLine(10, 4, ",",current.right, &Font_5x8);
			ssd1306PrintIntAtLine(10, 5, ",",current.rightExt, &Font_5x8);
			ssd1306Refresh(); */
		}
	}
	basicMove(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE){}
	// desactivate PID
    mainControlStopPidLoop();
	line_follower.active_state = FALSE;
	telemetersStop();
	motorsDriverSleep(ON);
}

//----------------------------------------------------------------
// Initialize data sensor to memorize the max and min value for each 5 sensors
void lineSensorsCalibration(void)
{
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
	basicMove(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE);
	basicMove(-80, 0, 150, 0);
	// -------------------------------------------------------------
	// Init line Sensor

	max_Floor.left=getLineSensorAdc(LINESENSOR_L);
	max_Floor.front=getLineSensorAdc(LINESENSOR_F);
	max_Floor.right=getLineSensorAdc(LINESENSOR_R);
	max_Floor.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
	max_Floor.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);
	memcpy(&min_Floor, &max_Floor, sizeof(ground_sensors_struct) );
	memcpy(&current, &min_Floor, sizeof(ground_sensors_struct) );

	while(hasMoveEnded() != TRUE)
	{

		current.left=getLineSensorAdc(LINESENSOR_L);
		current.front=getLineSensorAdc(LINESENSOR_F);
		current.right=getLineSensorAdc(LINESENSOR_R);
		current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
		current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);

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

	basicMove(40, 0, 30, 30);

	ssd1306ClearScreen(MAIN_AREA);
	while(hasMoveEnded() != TRUE)
	{  //=($A$1*A3+$B$1*B3+$C$1*C3+$D$1*D3+$E$1*E3)/(A3+B3+C3+D3+E3)

		current.left=getLineSensorAdc(LINESENSOR_L);
		current.front=getLineSensorAdc(LINESENSOR_F);
		current.right=getLineSensorAdc(LINESENSOR_R);
		current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
		current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);

		A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
		B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
		C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
		D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
		E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

		cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
		cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);
		ssd1306ClearRectAtLine(0, 1, 128);
		ssd1306ClearRectAtLine(0, 2, 128);
		ssd1306PrintIntAtLine(10, 1,  "Centre =  ", (uint16_t)cdg, &Font_5x8);
		ssd1306PrintIntAtLine(10, 2,  "milieu =  ", (uint16_t)D-B, &Font_5x8);
		ssd1306Refresh();
		HAL_Delay(50);
	}

	telemetersStop();
	motorsDriverSleep(ON);

	int joystick = expanderJoyFiltered();
	while (joystick!=JOY_LEFT)
	{

		joystick = expanderJoyFiltered();

		current.left=getLineSensorAdc(LINESENSOR_L);
		current.front=getLineSensorAdc(LINESENSOR_F);
		current.right=getLineSensorAdc(LINESENSOR_R);
		current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
		current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);

		A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
		B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
		C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
		D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
		E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

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

		ssd1306ClearScreen(MAIN_AREA);
		if (cdg>0)
		{
			ssd1306PrintIntAtLine(10, 1,  "Centre =  ", (uint16_t)cdg, &Font_5x8);
		} else
		{
			ssd1306PrintIntAtLine(10, 1,  "Centre =- ", (uint16_t)-cdg, &Font_5x8);
		}
		if (D>B)
		{
			ssd1306PrintIntAtLine(10, 2,  "milieu =  ", (uint16_t)cdg2, &Font_5x8);
		} else
		{
			ssd1306PrintIntAtLine(10, 2,  "milieu =- ", (uint16_t)-cdg2, &Font_5x8);
		}
		ssd1306Refresh();

	}
	lineSensorsStop();
}

// Check if there is no line
int lineFollowerStop()
{
	if (sensor.front < 200 &&
		sensor.left  < 200 &&
		sensor.right < 200 &&
		sensor.leftExt < 200 &&
		sensor.rightExt < 200)
	{
		return 1;
	}
	return 0;
}

int lineFollowerFigure()
{
	static int dist=0;
	if (sensor.front > 700 &&    // on est sur la ligne
		sensor.leftExt > 700 &&  // A gauche on a une ligne
		sensor.rightExt < 200)  // a droite on a pas de ligne
	{
        if (dist==0) dist = encoderGetDist(ENCODER_L)+35;
        if (encoderGetDist(ENCODER_L) > dist)
		{
			dist=0;
			return 1;
		}
	}
	return 0;
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{

//	double A,B,C,D,E;
	double cdg=0;
	double cdg2=0;

	lineSensorsInit();
	encodersInit();

//	positionControlSetPositionType(GYRO);
	mainControlSetFollowType(LINE_FOLLOW);

	lineSensorsStart();

	if (max_Floor.left-min_Floor.left< 100.0)
	{
		tone(a, 500);
		tone(b, 500);
		lineSensorsStop();
		telemetersStop();
		motorsDriverSleep(ON);
		return;
	}

	tone(c, 100);
	coef_Floor.left=100.0/(max_Floor.left-min_Floor.left);     //  100/(max_capteur-min_capteur) (0..100)
	coef_Floor.front=100.0/(max_Floor.front-min_Floor.front);
	coef_Floor.right=100.0/(max_Floor.right-min_Floor.right);
	coef_Floor.leftExt=100.0/(max_Floor.leftExt-min_Floor.leftExt);
	coef_Floor.rightExt=100.0/(max_Floor.rightExt-min_Floor.rightExt);

	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintIntAtLine(10, 0, "LEFT_EXT  =  ", (uint16_t) min_Floor.leftExt, &Font_5x8);
	ssd1306PrintIntAtLine(10, 1, "LEFT      =  ", (uint16_t) min_Floor.left, &Font_5x8);
	ssd1306PrintIntAtLine(10, 2, "FRONT --  =  ", (uint16_t) min_Floor.front, &Font_5x8);
	ssd1306PrintIntAtLine(10, 3, "RIGHT     =  ", (uint16_t) min_Floor.right, &Font_5x8);
	ssd1306PrintIntAtLine(10, 4, "RIGHT_EXT =  ", (uint16_t) min_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
	HAL_Delay(1000);
	tone(c, 100);

	//	HAL_Delay(500);

	line_follower.active_state = TRUE;
//	basicMove(0, 10000, MAXSPEED, MAXSPEED);
	motorsDriverSleep(ON);
	//	while(hasMoveEnded() != TRUE);
	char foreward = TRUE;
	int ii=0;

	while(expanderJoyFiltered()!=JOY_LEFT && foreward)
	{

		sensor.leftExt =1000*(getLineSensorAdc(LINESENSOR_EXT_L)-min_Floor.leftExt)/max_Floor.leftExt;
		sensor.left    =1000*(getLineSensorAdc(LINESENSOR_L)-min_Floor.left)/max_Floor.left;
		sensor.front   =1000*(getLineSensorAdc(LINESENSOR_F)-min_Floor.front)/max_Floor.front;
		sensor.right   =1000*(getLineSensorAdc(LINESENSOR_R)-min_Floor.right)/max_Floor.right;
		sensor.rightExt=1000*(getLineSensorAdc(LINESENSOR_EXT_R)-min_Floor.rightExt)/max_Floor.rightExt;
/*
		current.left=getLineSensorAdc(LINESENSOR_L);
		current.front=getLineSensorAdc(LINESENSOR_F);
		current.right=getLineSensorAdc(LINESENSOR_R);
		current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
		current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);

		A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
		B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
		C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
		D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
		E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;

		ssd1306ClearScreen(MAIN_AREA);
		ssd1306DrawLine(0,32, 127, 32);
		cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
		cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);
		*/
ii++;
	//	cdg=D-B;
		cdg=sensor.right-sensor.left;
		if (cdg<0)
		{
			cdg2=-sensor.left;
			//if (cdg<-500) basicMove(0, 10000, 500, 0);
		} else
		{
			cdg2=sensor.right;
			//if (cdg>500)basicMove(0, 10000, 500, 0);
		}

		//error=follow_control.follow_error*10;


	//	error=(int) cdg2/1000.0;

		ssd1306ClearScreen(MAIN_AREA);
		ssd1306PrintIntAtLine(2, 0, "ACTION    = ", _Action, &Font_5x8);
		ssd1306PrintIntAtLine(2, 1, "LEFT_EXT  = ", sensor.leftExt, &Font_5x8);
		ssd1306PrintIntAtLine(2, 2, "FRONT --  = ", cdg2, &Font_5x8);
		ssd1306PrintIntAtLine(2, 3, "RIGHT_EXT = ", sensor.rightExt, &Font_5x8);
		ssd1306PrintIntAtLine(2, 4, "Roue = ", (signed int) encoderGetDist(ENCODER_L), &Font_5x8);
		ssd1306Refresh();


		if (_Action>0)
		{
			line_follower.active_state = FALSE;
			positionControlSetPositionType(GYRO);
			mainControlSetFollowType(NO_FOLLOW);
			lineSensorsStop();

			// -----------------------------------------------------------------------
			// Condition to stop zhonx if no line
			// -----------------------------------------------------------------------
			if (_Action==1)
			{
				foreward = FALSE;
				basicMove(0, 30, MAXSPEED, 0);
				while(hasMoveEnded() != TRUE);
				motorsBrake();
			}

			if (_Action==2)
			{
				basicMove(0, 30, MAXSPEED, 0);
				basicMove(40, 0, 100, 0);
				while(hasMoveEnded() != TRUE);
				basicMove(-40, 0, 100, 0);
//				basicMove(360, 0, 200, 0);
				while(hasMoveEnded() != TRUE);
				lineSensorsStart();
				mainControlSetFollowType(LINE_FOLLOW);
//				basicMove(0, 10000, MAXSPEED, MAXSPEED);

				_Action=0;
			}

			//basicMove(0, 10000, MAXSPEED, MAXSPEED);
		}




		// -----------------------------------------------------------------------
		// Condition to stop if right priority
		// -----------------------------------------------------------------------
		//		if (((double)getLineSensorAdc(LINESENSOR_EXT_L)*1.2) > max_Floor.leftExt )
		//		{
		//			basicMove(0, 30, 30, 0);
		//			tone(c, 500);
		//			// capteur telemeter ON
		//			basicMove(0, 10000, MAXSPEED, 0);
		//		}

	}
	lineSensorsStop();
	telemetersStop();
	motorsDriverSleep(ON);
}

//-----------------------------------------------------------------------------
int EstAGauche()
{
	ground_sensors_struct current;
	current.left=getLineSensorAdc(LINESENSOR_L);
	current.front=getLineSensorAdc(LINESENSOR_F);
	current.right=getLineSensorAdc(LINESENSOR_R);
	current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
	current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);
	if ( (current.left>current.right)||
			(current.leftExt>max_Floor.leftExt/2))
		return 1;
	return 0;
}

//----------------------------------------------------------------------
// fonction pour asservir zhonx sur la ligne
//
void controlLoop(void)
{

//	double A,B,C,D,E;
	double cdg=0;
	double cdg2=0.0;

	sensor.leftExt =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_EXT_L)-min_Floor.leftExt)/max_Floor.leftExt;
	sensor.left    =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_L)-min_Floor.left)/max_Floor.left;
	sensor.front   =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_F)-min_Floor.front)/max_Floor.front;
	sensor.right   =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_R)-min_Floor.right)/max_Floor.right;
	sensor.rightExt=(unsigned int)1000*(getLineSensorAdc(LINESENSOR_EXT_R)-min_Floor.rightExt)/max_Floor.rightExt;
/*
	current.front=getLineSensorAdc(LINESENSOR_F);
	current.right=getLineSensorAdc(LINESENSOR_R);
	current.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
	current.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);

	A=(double)(current.leftExt-min_Floor.leftExt)/max_Floor.leftExt*1000;
	B=(double)(current.left-min_Floor.left)/max_Floor.left*1000;
	C=(double)(current.front-min_Floor.front)/max_Floor.front*1000;
	D=(double)(current.right-min_Floor.right)/max_Floor.right*1000;
	E=(double)(current.rightExt-min_Floor.rightExt)/max_Floor.rightExt*1000;
*/

	if (lineFollowerFigure()>0) _Action=2;
	if (lineFollowerStop()>0) _Action=1;

	//	cdg=(-1000*A-389*B+C+D*431+E*1000)/(A+B+C+D+E);
	//  cdg=(-500*A-194.5*B+C+D*215.5+E*500)/(A+B+C+D+E);
	cdg=sensor.right-sensor.left;
	if (cdg<0)
	{
		cdg2=-sensor.left;
	} else
	{
		cdg2=sensor.right;
	}
	line_follower.position = (cdg2)/1000.0;

}
void lineFollower_IT(void)
{
    if (line_follower.active_state == FALSE)
        return;
	// Rapide
	//	static int vitesse=0;

	controlLoop();


	//	if (follow_control.follow_error > 3.0 && vitesse==0)
	//	{
	//		// deceleration
	//		basicMove(0, 30, MAXSPEED, MINSPEED);
	//		vitesse=-1;
	//	}
	//	else if (follow_control.follow_error < 3.0 && vitesse==0)
	//	{
	//		// acceleration
	//		basicMove(0, 30, MINSPEED, MAXSPEED);
	//		vitesse=1;
	//	}
	//
	//	if (hasMoveEnded() == TRUE)
	//	{
	//		if (vitesse<0)
	//		{
	//			basicMove(0, 10000, MINSPEED, MINSPEED);
	//		}
	//		else if (vitesse>0)
	//		{
	//			basicMove(0, 10000, MAXSPEED, MAXSPEED);
	//		}
	//		vitesse=0;
	//	}
	// -----------------------------------------------------------------------
	// Condition to stop zhonx if no line
	// -----------------------------------------------------------------------
	//if ((double)getLineSensorAdc(LINESENSOR_F) < min_Floor.front *1.2 &&
		//	(double)getLineSensorAdc(LINESENSOR_L) < min_Floor.left *1.2 &&
		//	(double)getLineSensorAdc(LINESENSOR_R) < min_Floor.right *1.2 &&
		//	(double)getLineSensorAdc(LINESENSOR_EXT_L) < min_Floor.leftExt *1.2 &&
		//	(double)getLineSensorAdc(LINESENSOR_EXT_R) < min_Floor.rightExt *1.2)
	//{
	//	basicMove(0, 150, 250, 0);
	//}
}

