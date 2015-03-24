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

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"

/* Middleware declarations */

/* Declarations for this module */
#include "application/display/menu.h"

/*external fonctions */
extern void bluetoothTest();
extern void eepromTest ();
extern void encoderTest ();
extern void adxrs620Test ();
extern void mulimeterTest ();
extern void telemetersTest ();
extern void toneTest ();
extern void motorsTest ();
extern void lineSensorsTest ();
extern void straightControlTest ();
/*
 * pour cree un nouveau menu il suffit de
 * créer une nouvelle variable de type "menuItem"
 * la replre de la façon suivante :
 * menuItem name =
 * {
 * 		"nom du menu",
 * 		{
 * 			{"nom de la ligne 1",'type de l'argument', &(void*) poirteur_sur_l'argument},	// le type est soit 'i' pour un int, soit 'l' pour un long, soit 'm' pour un menu, soit 'f' pour une fonction
 * 			{"nom de la ligne 2",'type de l'argument', &(void*) poirteur_sur_l'argument},	// maximum 20 ligne. si cela ne suffi pas il faut en rajouter dans "menu.h" le typedef menuItem le nombre de case du tableau de "line"
 * 			{0,0,0} 					// la dernière ligne doit absolument être la précédente, cette ligne ne s'affichera pas mais est indispensable. /!\ cette ligne compte dans les 20 ligne du menu
 * 		}
 * }
 */
// les variables si dessous existe juste pour montrer comment modifier des variables
int toto=0;
long tata=0;
bool titi=false;
menuItem maze_menu={0,{{0,0,0}}};
menuItem parameters_menu=
{
		"menu parameters",
		{
				{"toto",'i', (void*)&toto},
				{"tata",'l', (void*)&tata},
				{"titi",'b', (void*)&titi},
				{0,0,0}
		}
};
menuItem tests_menu=
{
		"TEST MENU",
		{
				{"test bluetooth",'f', (void*)bluetoothTest},
				{"test multimeter",'f', (void*)mulimeterTest},
				{"test display",'f', (void*)ssd1306Test},
				{"test eeprom",'f', (void*)eepromTest},
				{"test encoders",'f', (void*)encoderTest},
				{"test joystick",'f', (void*)joystickTest},
				{"test gyroscope",'f', (void*)adxrs620Test},
				{"test telemeters",'f', (void*)telemetersTest},
				{"test beeper",'f', (void*)toneTest},
				{"test motors",'f', (void*)motorsTest},
				{"test line sensors",'f', (void*)lineSensorsTest},
				{0,0,0}
		}
};

menuItem mainMenu =
{
		"ZHONX III        V0.1",
		{
				{"Maze menu",'m',			(void*)&maze_menu },
				{"Parameters",'m',			(void*)&parameters_menu},
				{"Test menu",'m',			(void*)&tests_menu},
				{"Straight Test",'f',		(void*)&straightControlTest},
				{0,0,0}
		}
};

int menu(menuItem Menu)
{
	signed char line_screen=1;
	signed char line_menu=0;
	displayMenu(Menu,line_menu);
	ssd1306InvertArea(0, MARGIN, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
	ssd1306Refresh();
	while (true)
	{
		int joystick = expanderJoyState();
		// Exit Button JOYSTICK_LEFT
		switch (joystick)
		{
		case LEFT:
			antiBounceJoystick();
			return SUCCESS;
			break;
			// Joystick down
		case DOWN:
			//beeper
			antiBounceJoystick();
			if(Menu.line[line_menu+1].name!=null)
			{
				line_menu++;
				line_screen++;
				if(line_screen>MAX_LINE_SCREEN)
				{
					line_screen--;
					displayMenu(Menu,line_menu-(line_screen-1));
					ssd1306InvertArea(0, line_screen*MARGIN, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
					ssd1306Refresh();
				}
				else
				{
					menuHighlightedMove((line_screen-1)*ROW_HEIGHT+1, (line_screen)*ROW_HEIGHT);
				}
			}
			break;
		case UP :
			antiBounceJoystick();
			//beeper
			if(line_screen==1)
			{
				if(line_menu>0)
				{
					line_menu--;
					displayMenu(Menu,line_menu);
					ssd1306InvertArea(0, MARGIN, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
					ssd1306Refresh();
				}
			}
			else
			{
				line_menu--;
				line_screen--;
				menuHighlightedMove((line_screen+1)*ROW_HEIGHT-1, (line_screen)*ROW_HEIGHT);
			}
			break;
		case RIGHT :// Validate button joystick right
			//hal_beeper_beep(app_context.beeper, 4000, 10);
			antiBounceJoystick();
			switch(Menu.line[line_menu].type)
			{
			case 'b':
				modifyBoolParam(Menu.line[line_menu].name,(unsigned char*) Menu.line[line_menu].param);
				break;
			case 'i':
				modifyLongParam(Menu.line[line_menu].name,(long*)(int*)Menu.line[line_menu].param);
				break;
			case 'l':
				modifyLongParam(Menu.line[line_menu].name,(long*)Menu.line[line_menu].param);
				break;
			case 'm':
				menu(*(menuItem*)Menu.line[line_menu].param);
				break;
			case 'f':
				if (Menu.line[line_menu].param!=null)
					Menu.line[line_menu].param();
				break;
			default:
				break;
			}
			displayMenu(Menu,line_menu-(line_screen-1));
			ssd1306InvertArea(0,MARGIN*line_screen,HIGHLIGHT_LENGHT,HIGHLIGHT_HEIGHT);
			ssd1306Refresh();
			break;
			default:
				break;
		}
	}
	return -1;
}

void menuHighlightedMove(unsigned char y, unsigned char max_y)
{
	if (max_y > y)
	{
		ssd1306InvertArea(0, y-1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT*2);
		//		for ( ; y <= max_y; y++)
//		{
//			ssd1306InvertArea(0, y - 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
//			ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
////			if (y % 2)
//				ssd1306Refresh();
//		}
	}
	else
	{
		ssd1306InvertArea(0, y-HIGHLIGHT_HEIGHT+1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT*2);
//		for ( ; y >= max_y; y--)
//		{
//			ssd1306InvertArea(0, y + 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
//			ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
////			if (y % 2)
//				ssd1306Refresh();
//		}
	}
	ssd1306Refresh();
}

void displayMenu(menuItem menu,int line)
{
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
			ssd1306DrawString(110,i*MARGIN+MARGIN+1,">",&Font_3x6);
			break;
		case 'm':
			ssd1306DrawString(115,i*MARGIN+MARGIN+1,">",&Font_3x6);
			break;
		}
	}
	uint8_t nmbr_item = 0;
	while(menu.line[nmbr_item].name != null)
	{
		nmbr_item++;
	}
	if (nmbr_item>MAX_LINE_SCREEN)
	{
		int heightOneItem=54/nmbr_item;
		ssd1306DrawRect(123,heightOneItem*line+MARGIN,3,MAX_LINE_SCREEN*heightOneItem);
		ssd1306Refresh();
	}
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
		int joystick=expanderJoyState();
		switch (joystick)
		{
		case LEFT :
			// Wait until button is released
			antiBounceJoystick();
			return SUCCESS;
			break;

		case DOWN:
		case UP :
			antiBounceJoystick();
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

		case RIGHT:
			antiBounceJoystick();

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
		int joystick=expanderJoyState();
		switch (joystick)
		{
		case LEFT :
			antiBounceJoystick();
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
		case UP:
			antiBounceJoystick();
			//param_copy +=1;
			param_copy += (step*pow(10,collone));
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			break;
		case DOWN :
			antiBounceJoystick();
			param_copy -= (step*pow(10,collone));
			//param_copy -= 1;
			ssd1306ClearRect(0, 28, 164, 8);
			sprintf(str, "%10i", (int)param_copy);
			ssd1306DrawString(0, 28, str, &Font_8x8);
			ssd1306Refresh();
			break;
		case RIGHT :
			antiBounceJoystick();
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
