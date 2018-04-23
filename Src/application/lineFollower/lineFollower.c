/**************************************************************************/
/*!
    @file    line_follower.c
    @author   BM Pacabot.com
    @date     05 May 2015
    @version  0.01
 */
/**************************************************************************/
/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <string.h>

/* Application declarations */
#include "application/lineFollower/lineFollower.h"
#include "application/statistiques/statistiques.h"

/* Middleware declarations */
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/moves/basicMoves/basicMoves.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"

/* Peripheral declarations */
#include <peripherals/bluetooth/bluetooth.h>
#include <peripherals/display/smallfonts.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/encoders/ie512.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/lineSensors/lineSensors.h>
#include <peripherals/motors/motors.h>
#include <peripherals/telemeters/telemeters.h>
#include <peripherals/tone/tone.h>


line_follower_struct line_follower;
calibrate_sensors_struct max_Floor;	//global data to memorize maximum value of sensors
calibrate_sensors_struct coef_Floor;	//global data to memorize coeff value (0..1000]
calibrate_sensors_struct min_Floor;	//global data to memorize minimum value of sensors
int sensor[5];


//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
int line_speed = 40;
int line_length = 200000;

void lineSensorSendBluetooth(void)
{
	calibrate_sensors_struct current;

	int i=0;
	int GO=0;

	lineSensorsInit();

	positionControlSetPositionType(ENCODERS);
	mainControlSetFollowType(NO_FOLLOW);

	lineSensorsStart();

	tone(e, 500);
	basicMove(40, 1, 10, 0);
	while(hasMoveEnded() != TRUE);
	basicMove(-80, 1, 25, 0);
	max_Floor.left = getLineSensorAdc(LINESENSOR_L);
	max_Floor.front = getLineSensorAdc(LINESENSOR_F);
	max_Floor.right = getLineSensorAdc(LINESENSOR_R);
	max_Floor.leftExt = getLineSensorAdc(LINESENSOR_EXT_L);
	max_Floor.rightExt = getLineSensorAdc(LINESENSOR_EXT_R);
	memcpy(&min_Floor, &max_Floor, sizeof(max_Floor) );
	memcpy(&current, &min_Floor, sizeof(min_Floor) );

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
	}
	basicMove(40, 0, 100, 0);
	while(hasMoveEnded() != TRUE);
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

	positionControlSetPositionType(ENCODERS);
	mainControlSetFollowType(NO_FOLLOW);

	HAL_Delay(1000);

	double cdg=0;
	double cdg2=0;
	double A,B,C,D,E;
	calibrate_sensors_struct current;

	lineSensorsStart();

	tone(e, 500);
	basicMove(CALIB_ANGL / 2, 0, 100, 0);
	while(hasMoveEnded() != TRUE);
	basicMove(-CALIB_ANGL, 0, 150, 0);
	// -------------------------------------------------------------
	// Init line Sensor

	max_Floor.left=getLineSensorAdc(LINESENSOR_L);
	max_Floor.front=getLineSensorAdc(LINESENSOR_F);
	max_Floor.right=getLineSensorAdc(LINESENSOR_R);
	max_Floor.leftExt=getLineSensorAdc(LINESENSOR_EXT_L);
	max_Floor.rightExt=getLineSensorAdc(LINESENSOR_EXT_R);
	memcpy(&min_Floor, &max_Floor, sizeof(max_Floor) );
	memcpy(&current, &min_Floor, sizeof(max_Floor) );

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

	basicMove(CALIB_ANGL / 2, 0, 30, 30);

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
		ssd1306PrintIntAtLine(10, 1,  "Centre =  ", cdg, &Font_5x8);
		ssd1306PrintIntAtLine(10, 2,  "milieu = ", cdg2, &Font_5x8);
		ssd1306Refresh();

	}
	lineSensorsStop();
}
//---------------------------------------------------------------------
int lineFollowerFigure()
{
	static int dist=0;
	int tmp = encoderGetDist(ENCODER_L);
	if (sensor[2] > 600 &&    // on est sur la ligne
		sensor[0] > 600 &&  // A gauche on a une ligne
		sensor[4] < 200)   // a droite on n' a pas de ligne
	{
        if (dist==0) dist = tmp + 30;  // Au dela d'une certaine distance on n'est pas sur la double ligne
        if (tmp > dist + 35)
		{
			dist=0;
		} else if (tmp > dist)
		{
			dist=0;
			return 1;
		}
	}
	return 0;
}
void line_print_info()
{
	double cdg=0;
	double cdg2=0;
	int max_i = 2;
	int max= sensor[max_i];
	for (int i = 1; i < 4; ++i)
	{
		if(max < sensor[i])
		{
			max = sensor[i];
			max_i = i;
		}
	}

	cdg=sensor[max_i + 1]-sensor[max_i - 1];
	if (cdg<0)
	{
		cdg2=-sensor[max_i - 1];
	} else
	{
		cdg2=sensor[max_i + 1];
	}
	cdg2 = (max_i - 2) * 1000 + (cdg2) ;
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintIntAtLine(2, 1, "MAX i = ",(signed int) max_i, &Font_5x8);
	ssd1306PrintIntAtLine(2, 2, "suivi --  = ",(signed int) cdg2, &Font_5x8);
	ssd1306PrintIntAtLine(2, 4, "Roue = ", (signed int) (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2, &Font_5x8);
	ssd1306Refresh();
}

void test_line_sensors()
{
	lineSensorsInit();
	lineSensorsStart();
	line_follower.active_state = TRUE;
	int joystick = expanderJoyFiltered();
	basicMove(0, 30, line_speed, 0);
	while (joystick!=JOY_LEFT)
	{
		line_print_info();
		joystick = expanderJoyFiltered();
	}
	lineSensorsStop();
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{

	lineSensorsInit();
	encodersInit();
	lineFollowControlInit();

	if (max_Floor.left-min_Floor.left< 100.0)
	{
		tone(a, 500);
		tone(b, 500);

		min_Floor.leftExt=200;
		min_Floor.left=200;
		min_Floor.front=200;
		min_Floor.right=200;
		min_Floor.rightExt=200;
		max_Floor.leftExt=3200;
		max_Floor.left=3200;
		max_Floor.front=3200;
		max_Floor.right=3200;
		max_Floor.rightExt=3200;

		motorsDriverSleep(ON);
		return;
	}
	encodersReset();
	mainControlSetFollowType(LINE_FOLLOW);

	lineSensorsStart();

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
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintIntAtLine(10, 0, "LEFT_EXT  =  ", (uint16_t) max_Floor.leftExt, &Font_5x8);
	ssd1306PrintIntAtLine(10, 1, "LEFT      =  ", (uint16_t) max_Floor.left, &Font_5x8);
	ssd1306PrintIntAtLine(10, 2, "FRONT --  =  ", (uint16_t) max_Floor.front, &Font_5x8);
	ssd1306PrintIntAtLine(10, 3, "RIGHT     =  ", (uint16_t) max_Floor.right, &Font_5x8);
	ssd1306PrintIntAtLine(10, 4, "RIGHT_EXT =  ", (uint16_t) max_Floor.rightExt, &Font_5x8);
	ssd1306Refresh();
	HAL_Delay(1000);
	tone(c, 100);


	line_follower.active_state = TRUE;

	HAL_Delay(500);
	basicMoveStraight(line_length,line_speed, line_speed, 2000.0);
//	motorsDriverSleep(ON);

	int ii=0;

	while(expanderJoyFiltered()!=JOY_LEFT && hasMoveEnded() == false)
	{
		line_print_info();
	}
	mainControlSetFollowType(NO_FOLLOW);
	basicMoveStraight(30,line_speed, 0, 4000.0);        // on s'arrete
	line_follower.active_state = FALSE;
	lineSensorsStop();
	basicMove(0, 30, line_speed, 0);
	while(hasMoveEnded() != TRUE);
	motorsBrake();

	lineSensorsStop();
	telemetersStop();
	motorsDriverSleep(ON);
}

//-----------------------------------------------------------------------------
int EstAGauche()
{
	calibrate_sensors_struct current;
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
	double cdg=0;
	double cdg2=0.0;

	sensor[0] =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_EXT_L)-min_Floor.leftExt)/max_Floor.leftExt;
	sensor[1]    =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_L)-min_Floor.left)/max_Floor.left;
	sensor[2]   =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_F)-min_Floor.front)/max_Floor.front;
	sensor[3]   =(unsigned int)1000*(getLineSensorAdc(LINESENSOR_R)-min_Floor.right)/max_Floor.right;
	sensor[4]=(unsigned int)1000*(getLineSensorAdc(LINESENSOR_EXT_R)-min_Floor.rightExt)/max_Floor.rightExt;
	int max_i = 2;
	int max= sensor[max_i];
	for (int i = 1; i < 4; ++i)
	{
		if(max < sensor[i])
		{
			max = sensor[i];
			max_i = i;
		}
	}

	cdg=sensor[max_i + 1]-sensor[max_i - 1];
	if (cdg<0)
	{
		cdg2=-sensor[max_i - 1];
	} else
	{
		cdg2=sensor[max_i + 1];
	}
	line_follower.position = (max_i - 2) + (cdg2)/1000.0 ;

}
void lineFollower_IT(void)
{
    if (line_follower.active_state == FALSE)
    {
        return;
    }

	controlLoop();
}

