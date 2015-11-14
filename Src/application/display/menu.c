/*
 *  menu.c
 *
 *  Created on: 4 déc. 2014
 *      Author: colin
 */
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

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/multimeter/multimeter.h"
#include "peripherals/tone/tone.h"

/* Middleware declarations */
#include "middleware/display/icons.h"
#include "middleware/display/pictures.h"
#include "middleware/settings/settings.h"
#include "middleware/cmdline/cmdline_parser.h"

/* Declarations for this module */
#include "application/display/menu.h"

/*external functions */
extern void bluetoothTest();
extern void eepromTest ();
extern void encoderTest ();
extern void adxrs620Test ();
extern void mulimeterTest ();
extern void telemetersTest ();
extern void toneTest ();
extern void motorsTest ();
extern void lineSensorsTest ();
extern void lineFollower();
extern void mainControlTest ();
extern int 	lineSensorsCalibration (void);
extern void testTelemeterDistance();
extern void maze();
extern void testWallsSensors();
extern void followWallTest(void);
extern void followLineTest(void);
extern void rotateTest(void);
extern void curveRotateTest(void);
extern void expenderLedTest();
extern int	wallSensorsCalibrationFront(void);
extern int	wallSensorsCalibrationDiag (void);

/*
 * to create a new menu you have to create a new variable of type "const menuItem" like this :
 * const menuItem name =
 * {
 * 		"menu name",
 * 		{
 * 			{"line 1 name ",'type of argument', &(void*) pointeur_on_argument},		// 'type of argument' could be 'i' for int, or 'l' for long, or 'm' for  a menu, or 'f' for a function
 * 			{"line 1 name ",'type of argument', &(void*) pointeur_on_argument},		// maximum 20 line. if it's not enough you must add in "menu.h". for that you have to modify "MAX_LINE_IN_MENU"
 * 			{0,0,0} 					// the last ligne must be this one, this line will be not print but indispensable. /!\ cette ligne compte dans les 20 ligne du menu
 * 		}
 * }
 */

float toto=4.0;
float titi=4.0;
float tata=4.0;


const menuItem testGraphicMenu =
{
		"test graphic menu",
		{
				{"Default accel :",'a',			(void*)&toto},
				{"Max speed dist:",'a',			(void*)&titi},
				{"Default accel :",'a',			(void*)&tata},
				{"graphique",'g',null},
				{(char*)NULL,	0,				NULL}
		}
};

const menuItem maze_menu=
{
		"maze menu",
		{
				{"new maze",'f',		(void*)maze},
				//			{"test maze",'f',		(void*)test_maze},
				{"calibration",'b',		(void*)&zhonxSettings.calibration_enabled},
				{"color finish",'b',	(void*)&zhonxSettings.color_sensor_enabled},
				{"x finish",'i',		(void*)&zhonxSettings.maze_end_coordinate.x},
				{"y finish",'i',		(void*)&zhonxSettings.maze_end_coordinate.y}
		}

};

const menuItem follower_menu=
{
		"line menu",
		{
				{"line follower",'f',	(void*)lineFollower},
				{"calibration",'f',		(void*)lineSensorsCalibration},
		}
};


const menuItem parameters_menu=
{
		"Parameters menu",
		{
				{"calibration front",'f',(void*)wallSensorsCalibrationFront},
				{"calibration diag",'f',(void*)wallSensorsCalibrationDiag}
		}
};
const menuItem peripheral_test_menu=
{
		"peripherals test menu"
};
const menuItem tests_menu=
{
		"UNIT TEST",
		{
				{"distantce",		'f', (void*)testTelemeterDistance},
				{"wall sensor",		'f', (void*)testWallsSensors},
				{"bluetooth",		'f', (void*)bluetoothTest},
				{"multimeter",		'f', (void*)mulimeterTest},
				{"display",			'f', (void*)ssd1306Test},
				{"eeprom",			'f', (void*)eepromTest},
				{"encoders",		'f', (void*)encoderTest},
				{"joystick",		'f', (void*)joystickTest},
				{"gyroscope",		'f', (void*)adxrs620Test},
				{"telemeters",		'f', (void*)telemetersTest},
				{"beeper",			'f', (void*)toneTest},
				{"motors",			'f', (void*)motorsTest},
				{"line sensors",	'f', (void*)lineSensorsTest},
				{"Expender LEDs",	'f', (void*)expenderLedTest},
				{0,0,0}
		}
};

const menuItem control_menu=
{
		"control menu",
		{
				{"control test",'f',		(void*)mainControlTest},
				{"follow the wall",'f',		(void*)followWallTest},
				{"follow the line",'f',		(void*)followLineTest},
				{"rotate",'f',				(void*)rotateTest},
				{"curve rotate",'f',		(void*)curveRotateTest},
		}
};

const menuItem mainMenu =
{
#ifdef MEDDLE
		"ZHONX III meddle V0.2",
#else
		"ZHONX III dark   V0.2",
#endif
		{
			//	{"telemeters calibration",'f',		(void*)telemeterFrontCalibration},
				{"Maze menu",'m',			(void*)&maze_menu},
				{"Unit tests",'m',			(void*)&tests_menu},
				{"Control menu",'m',		(void*)&control_menu},
				{"Line menu",'m', 			(void*)&follower_menu},
				{"Parameters menu",'m',		(void*)&parameters_menu},
				{"Test graph",'m',			(void*)&testGraphicMenu},
				{0,0,0}
		}
};

extern I2C_HandleTypeDef hi2c1;

int menu(const menuItem Menu)
{
	signed char line_screen = 1;
	signed char line_menu = 0;
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
	displayMenu(Menu, line_menu);
	ssd1306InvertArea(0, MARGIN, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
	ssd1306Refresh();
	while (true)
	{
		int joystick = expanderJoyFiltered();
		killOnLowBattery();
		switch (joystick)
		{
		case JOY_LEFT:
			return SUCCESS;
			break;
			// Joystick down
		case JOY_DOWN:
			//beeper
			if (Menu.line[line_menu + 1].name != null)
			{
				line_menu++;
				if (line_screen >= MAX_LINE_SCREEN)
				{
					displayMenu(Menu, line_menu - (line_screen - 1));
					ssd1306InvertArea(0, line_screen * MARGIN,
							HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
					ssd1306Refresh();
				}
				else
				{
					line_screen++;
					menuHighlightedMove((line_screen - 1) * ROW_HEIGHT + 1,
							(line_screen) * ROW_HEIGHT);
				}
			}
			else
			{
				line_menu = 0;
				line_screen = 1;displayMenu(Menu, line_menu - (line_screen - 1));
				ssd1306InvertArea(0, line_screen * MARGIN,
						HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
				ssd1306Refresh();
			}
			break;
		case JOY_UP:
			//beeper
			if (line_menu > 0)
			{
				line_menu--;
				if (line_screen <= 1)
				{
					displayMenu(Menu, line_menu);
					ssd1306InvertArea(0, MARGIN, HIGHLIGHT_LENGHT,
							HIGHLIGHT_HEIGHT);
					ssd1306Refresh();
				}
				else
				{
					line_screen--;
					menuHighlightedMove((line_screen + 1) * ROW_HEIGHT - 1,
							(line_screen) * ROW_HEIGHT);
				}
			}
			else
			{
				while(Menu.line[line_menu+1].name != null)
				{
					line_menu++;
				}
				if (line_menu < MAX_LINE_SCREEN-1)
				{
					displayMenu(Menu, 0);
					line_screen = line_menu;
				}
				else
				{
					line_screen = MAX_LINE_SCREEN;
					displayMenu(Menu, line_menu-(MAX_LINE_SCREEN-1));
				}
				ssd1306InvertArea(0, MARGIN * line_screen, HIGHLIGHT_LENGHT,
									HIGHLIGHT_HEIGHT);
				ssd1306Refresh();
			}
			break;
		case JOY_RIGHT: // Validate button joystick right
			//hal_beeper_beep(app_context.beeper, 4000, 10);
			switch (Menu.line[line_menu].type)
			{
			case 'b':
				modifyBoolParam(Menu.line[line_menu].name,
						(unsigned char*) Menu.line[line_menu].param);
				break;
			case 'i':
				modifyLongParam(Menu.line[line_menu].name,
						(long*) (int*) Menu.line[line_menu].param);
				break;
			case 'l':
				modifyLongParam(Menu.line[line_menu].name,
						(long*) Menu.line[line_menu].param);
				break;
			case 'm':
				menu(*(const menuItem*) Menu.line[line_menu].param);
				break;
			case 'f':
				if (Menu.line[line_menu].param != null)
				{
					ssd1306ClearScreen();
					ssd1306Refresh();
					Menu.line[line_menu].param();
				}
				break;
			case 'g':
				graphMotorSettings(
						(float*) Menu.line[line_menu - 3].param,
						(float*) Menu.line[line_menu - 2].param,
						(float*) Menu.line[line_menu - 1].param);
				break;
			default:
				break;
			}
			displayMenu(Menu, line_menu - (line_screen - 1));
			ssd1306InvertArea(0, MARGIN * line_screen, HIGHLIGHT_LENGHT,
					HIGHLIGHT_HEIGHT);
			ssd1306Refresh();
			break;
			default:
				break;
		}
		cmdline_parse();
	}
	return -1;
}

void menuHighlightedMove(unsigned char y, unsigned char max_y)
{
	if (max_y > y)
	{
		//		ssd1306InvertArea(0, y-1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT*2);
		for ( ; y <= max_y; y++)
		{
			ssd1306InvertArea(0, y - 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
			ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
			ssd1306Refresh();
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		}
	}
	else
	{
		//		ssd1306InvertArea(0, y-HIGHLIGHT_HEIGHT+1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT*2);
		for ( ; y >= max_y; y--)
		{
			ssd1306InvertArea(0, y + 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
			ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
			ssd1306Refresh();
			HAL_Delay(5);
			while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);
		}
	}
//	ssd1306Refresh();
}

void displayMenu(const menuItem menu,int line)
{
	//char str[5];
	ssd1306ClearScreen();
	ssd1306DrawString(0,0,menu.name,&Font_5x8);
	ssd1306DrawLine(0,8,128,8);
	for (int i=0;i<MAX_LINE_SCREEN;i++)
	{
		if(menu.line[i].name!=null)
			ssd1306DrawString(0,MARGIN*i+MARGIN+1,menu.line[line+i].name,&Font_5x8);
		switch (menu.line[line+i].type)
		{
		case 'b':
			if(*((bool*)menu.line[i+line].param)==true)
				ssd1306DrawString(90,MARGIN*i+MARGIN+1,"yes",&Font_5x8);
			else
				ssd1306DrawString(90,MARGIN*i+MARGIN+1,"no",&Font_5x8);
			break;
		case 'i':
			ssd1306PrintInt(90,MARGIN*i+MARGIN+1," ",*((unsigned int*)menu.line[i+line].param),&Font_3x6);
			break;
		case 'l':
			ssd1306PrintInt(90,MARGIN*i+MARGIN+1," ",*((unsigned long*)menu.line[i+line].param),&Font_3x6);
			break;
		case 'f':
			ssd1306DrawString(110,i*MARGIN+MARGIN+1,"->",&Font_3x6);
			break;
		case 'm':
			ssd1306DrawString(115,i*MARGIN+MARGIN+1,">",&Font_3x6);
			break;
			//		case 'a':
			//			sprintf(str,"%f.2",*(float*)menu.line[i+line].param);
			//			ssd1306DrawString(110,i*MARGIN+MARGIN+1,str,&Font_3x6);
		}
	}
	uint8_t nmbr_item = 0;
	while(menu.line[nmbr_item].name != null)
	{
		nmbr_item++;
	}
	if (nmbr_item>MAX_LINE_SCREEN)
	{
		//int heightOneItem=54/nmbr_item;
		ssd1306DrawRect(123,(54*line)/nmbr_item+MARGIN,3,(54*MAX_LINE_SCREEN)/nmbr_item);
	}
	ssd1306Refresh();
}
int modifyBoolParam( char *param_name, unsigned char *param)
{
	char str[4];
	bool param_copy = (bool)*param;

	ssd1306ClearScreen();

	// Write the parameter name
	ssd1306DrawString(0, 0,param_name, &Font_5x8);
	ssd1306DrawLine(0, 9, 128, 9);

	if (param_copy == true)
	{
		sprintf(str, "YES");
	}
	else
	{
		sprintf(str, "NO");
	}
	ssd1306DrawString(0, 28, str, &Font_8x8);
	ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
	ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
	ssd1306Refresh();

	while (1)
	{
		int joystick=expanderJoyFiltered();
		switch (joystick)
		{
		case JOY_LEFT :
			return SUCCESS;
			break;

		case JOY_DOWN:
		case JOY_UP :
			if (param_copy == true)
			{
				param_copy = false;
				sprintf(str, "NO");
			}
			else
			{
				param_copy = true;
				sprintf(str, "YES");
			}
			ssd1306ClearRect(0, 28, 164, 8);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			break;

		case JOY_RIGHT:

			*param = param_copy;
			ssd1306ClearScreen();
			ssd1306Refresh();
			return SUCCESS;
			break;
		}
	}
	return SUCCESS;
}


int modifyLongParam( char *param_name,long *param)
{
	int step=1;
	char str[40];
	long param_copy = *param;
	char collone=0;
	ssd1306ClearScreen();

	// Write the parameter name
	ssd1306DrawString(0, 0,param_name, &Font_5x8);
	ssd1306DrawLine(0, 9, 128, 9);

	sprintf(str, "%10i", (int)param_copy);
	ssd1306DrawString(0, 28, str, &Font_8x8);
	ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
	ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
	ssd1306DrawString((10-collone)*8,20,"^",&Font_8x8);
	ssd1306DrawString((10-collone)*8,36,"v",&Font_8x8);
	ssd1306Refresh();

	while (1)
	{
		// Exit Button
		int joystick=expanderJoyFiltered();
		switch (joystick)
		{
		case JOY_LEFT :
			if (collone==10)
				return SUCCESS;
			else
			{
				collone++;
				ssd1306ClearRect(0,20,128,8);
				ssd1306ClearRect(0,36,128,8);
				ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
				ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
				ssd1306Refresh();
			}
			break;
		case JOY_UP:

			//param_copy +=1;
			param_copy += (step*pow(10,collone));
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			break;
		case JOY_DOWN :

			param_copy -= (step*pow(10,collone));
			//param_copy -= 1;
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			break;
		case JOY_RIGHT :
			if(collone==0)
			{
				*param = param_copy;
				ssd1306Refresh();
				return SUCCESS;
			}
			else
			{
				collone--;
				ssd1306ClearRect(0,20,128,8);
				ssd1306ClearRect(0,36,128,8);
				ssd1306DrawString((9-collone)*9,20,"^",&Font_8x8);
				ssd1306DrawString((9-collone)*9,36,"v",&Font_8x8);
				ssd1306Refresh();
			}
			break;
		default:
			break;
		}
	}

	return SUCCESS;
}

void graphMotorSettings (float *acceleration, float *maxSpeed, float *deceleration)
{
	int number_value=0;
	float* values[3]={acceleration,maxSpeed,deceleration};
	while(true)
	{
		printGraphMotor ( *acceleration, *maxSpeed, *deceleration);
		switch (expanderJoyFiltered())
		{
		case JOY_LEFT:
			if (number_value <= 0)
			{
				return;
			}
			else
			{
				number_value--;
			}
			break;
		case JOY_DOWN:
			*(values[number_value])-=0.1;
			break;
		case JOY_UP:
			*(values[number_value])+=0.1;
			break;
		case JOY_RIGHT:
			if (number_value >= 2)
			{
				return;
			}
			else
			{
				number_value++;
			}
			break;
		default:
			break;
		}
	}
}
void printGraphMotor (float acceleration, float maxSpeed, float deceleration)
{
	char str[10];
	ssd1306ClearScreen();
	char point1[2]={(char)((0-maxSpeed)/(0-acceleration)),64-(char)maxSpeed};
	char point2[2]={(char)(128-(0-maxSpeed)/(0-deceleration)),64-(char)(maxSpeed)};
	ssd1306DrawLine(0,64,point1[0],point1[1]);
	ssd1306DrawLine(point1[0],point1[1],point2[0],point2[1]);
	ssd1306DrawLine(point2[0],point1[1],128,64);

	sprintf(str,"%.2fM.S^2",acceleration);
	ssd1306DrawString((0+point1[0])/2+2,(64+point1[1])/2+2,str,&Font_3x6);

	sprintf(str,"%.2fM.S",maxSpeed);
	ssd1306DrawString((point1[0]+point2[0])/2,(point1[1]+point2[1])/2,str,&Font_3x6);

	sprintf(str,"%.2fM.S^2",deceleration);
	ssd1306DrawString((point2[0]+128)/2-27,(point2[1]+64)/2,str,&Font_3x6);
	ssd1306Refresh();
}
void welcomDisplay()
{

	ssd1306ClearScreen();

	ssd1306DrawBmp(Pacabot_bmp, 1, 1, 128, 40);
	ssd1306Refresh();
	while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

	HAL_Delay(5);
	for (int i = 0; i <= 100; i+=1)
	{
		ssd1306ProgressBar(10, 35, i);
		ssd1306Refresh();
		HAL_Delay(5);
	}
	ssd1306ClearScreen();
	ssd1306DrawBmp(five_years, 1, 1, 128, 54);
	ssd1306Refresh();
	HAL_Delay(300);
}

void powerOffConfirmation()
{
	unsigned char confirm = FALSE;
	modifyBoolParam("TURN POWER OFF ?",&confirm);
	if (confirm == TRUE)
	{
		modifyBoolParam("SAVE PARAM ?",&confirm);
		if (confirm == TRUE)
		{
//			save_setting();
		}
		halt();
		while(1);
	}
}
void killOnLowBattery()
{
	if(multimeter.vbat.value < (BATTERY_LOWER_VOLTAGE_NO_LOAD)*1000)
	{
		tone(A2,500);
		HAL_Delay(400);
		halt();
	}
}
