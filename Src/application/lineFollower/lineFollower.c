/**************************************************************************/
/*!
    @file     line_follower.c
    @author   BM Pacabot.com
    @date     05 May 2015
    @update   Colin Roubaud
    @date     08 may 2018
    @version  0.02
 */
/**************************************************************************/
/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"
#include "stm32f4xx_hal.h"
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

//calibrate_sensors_struct max_Floor;    /*!< global data to memorize maximum value of sensors */
//calibrate_sensors_struct coef_Floor;   /*!< global data to memorize coeff value [0..1000]    */
//calibrate_sensors_struct min_Floor;    /*!< global data to memorize minimum value of sensors */
//int sensor[5];


//__IO uint16_t ADC1ConvertedValues[2] = {0};
//__IO uint16_t ADC3ConvertedValues[3] = {0};

GPIO_InitTypeDef GPIO_InitStruct;
int line_speed = 40;
int line_length = 200000;

void line_print_info(void)
{
	int line_pos = getLinePos();
	ssd1306ClearScreen(MAIN_AREA);
	ssd1306PrintfAtLine(2, 2, &Font_5x8, "suivi --  = %d", line_pos);
	ssd1306PrintfAtLine(2, 4, &Font_5x8, "Roue = %d", (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2);
	ssd1306Refresh();
}

void test_line_sensors(void)
{
	lineSensorsInit();
	lineSensorsStart();

	FloorSensorsCalib calib = floorSensorCalib();
	int joystick = expanderJoyFiltered();
	basicMove(0, 30, line_speed, 0);
	while (joystick != JOY_LEFT)
	{
		line_print_info();
		joystick = expanderJoyFiltered();
	}
	lineSensorsStop();
	motorsDriverSleep(ON);
}

//---------------------------------------------------------------------
// Intelligent function to manage zhonx on the line path
void lineFollower(void)
{

	lineSensorsInit();
	encodersInit();
	lineFollowControlInit();


	lineSensorsStart();


	encodersReset();
	mainControlSetFollowType(LINE_FOLLOW);


	tone(c, 100);


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
	lineSensorsStop();
	basicMove(0, 30, line_speed, 0);
	while(hasMoveEnded() != TRUE);
	motorsBrake();

	lineSensorsStop();
	telemetersStop();
	motorsDriverSleep(ON);
}
