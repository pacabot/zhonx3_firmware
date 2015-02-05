/*
 * menu_colin.c
 *
 *  Created on: 4 déc. 2014
 *      Author: colin
 */
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "config/basetypes.h"

#include "peripherals/display/menu.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "stm32f4xx.h"

#include "config/basetypes.h"
#include "config/errors.h"
#include "config/config.h"
#include "peripherals/expander/pcf8574.h"


#include "peripherals/tests/tests.h"


//long pow (long nombre, long pow_);


/*
 * pour cree un nouveau menu il suffit de
 * créer une nouvelle variable de type "menuItem"
 * la replre de la façon suivente :
 * menuItem name =
 * {
 * 		"nom du menu",
 * 		{
 * 			{"nom de la ligne 1",'type de l'argument', &(void*) poirteur_sur_l'argument},	// le type est soit 'i' pour un int, soit 'l' pour un long, soit 'm' pour un menu, soit 'f' pour une fonction
 * 			{"nom de la ligne 1",'type de l'argument', &(void*) poirteur_sur_l'argument},	// maximum 20 ligne. si cela ne suffi pas il faut en rajouter dans "menu.h" le typedef menuItem le nombre de case du tableau de "line"
 * 			{0,0,0} 					// la dernière ligne doit absolument être la précédente, cette ligne ne s'affichera pas mais est indispensable. /!\ cette ligne compte dans les 20 ligne du menu
 * 		}
 * }
 */
int toto=0;
long tata=0;
bool titi=false;
void hal_ui_clear_scr()
{
	ssd1306ClearScreen();
}
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
		"test menu",
		{
				{"test gyro",'f',(void*)test_Gyro},
				{"test batteries",'f', (void*)test_Vbat},
				{"test encoder",'f', (void*)test_Encoders},
				//{"test beeper",'f', (void*)test_Beeper},
				//{"test expander",'f', (void*)test_Expander},
				{"test OLED",'f', (void*)test_Oled},
				{"test line sensors",'f', (void*)test_LineSensors},
				{0,0,0}
		}
};
menuItem mainMenu =
{
		"ZHONX III                 V1.0",
		{
			{"Maze menu",'m',			(void*)&maze_menu },
			{"Parameters",'m',			(void*)&parameters_menu},
			{"test menu",'m',			(void*)&tests_menu},
			{0,0,0}
		}
};

int menu(menuItem Menu)
{
	signed char line_screen=1;
	signed char line_menu=0;
	affiche_menu(Menu,line_menu);
	ssd1306InvertArea(0, 10, 120, 10);
	ssd1306Refresh();
	while (true)
	{
		int joystick = Expander_Joy_State();
		// Exit Button JOYSTICK_LEFT
		switch (joystick)
		{
			case LEFT:
				anti_rebonds();
				return SUCCESS;
				break;
		// Joystick down
			case DOWN:
				//beeper
				anti_rebonds();
				if(Menu.line[line_menu+1].name!=null)
				{
					line_menu++;
					line_screen++;
					if(line_screen>MAX_LINE_SCREEN)
					{
						line_screen--;
						affiche_menu(Menu,line_menu-(line_screen-1));
						ssd1306InvertArea(0, line_screen*10, 120, 10);
						ssd1306Refresh();
					}
					else
					{
						menu_animate((line_screen-1)*10+1, (line_screen)*10);
					}
				}
				break;
			case UP :
				anti_rebonds();
				//beeper
				if(line_screen==1)
				{
					if(line_menu>0)
					{
						line_menu--;
						affiche_menu(Menu,line_menu);
						ssd1306InvertArea(0, 10, 120, 10);
						ssd1306Refresh();
					}
				}
				else
				{
					line_menu--;
					line_screen--;
					menu_animate((line_screen+1)*10-1, (line_screen)*10);
				}
				break;
			case RIGHT :// Validate button joystick right
				//hal_beeper_beep(app_context.beeper, 4000, 10);
				anti_rebonds();
				switch(Menu.line[line_menu].type)
				{
					case 'b':
						modify_bool_param(Menu.line[line_menu].name,(unsigned char*) Menu.line[line_menu].param);
						break;
					case 'i':
						modify_long_param(Menu.line[line_menu].name,(long*)(int*)Menu.line[line_menu].param);
						break;
					case 'l':
						modify_long_param(Menu.line[line_menu].name,(long*)Menu.line[line_menu].param);
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
				affiche_menu(Menu,line_menu-(line_screen-1));
				ssd1306InvertArea(0,10*line_screen,120,10);
				ssd1306Refresh();
				break;
			default:
				break;
			}
		}
	return -1;
}
void anti_rebonds ()
{
	unsigned long int time_base = HAL_GetTick();
	do
	{
		if (Expander_Joy_State()!=0)
			time_base = HAL_GetTick();
	}while (time_base!=(HAL_GetTick()-200));
}
void menu_animate(unsigned char y, unsigned char max_y)
{
//	unsigned char y=y_;
//	unsigned char max_y=max_y_;

    if (max_y > y)
    {
        for ( ; y <= max_y; y++)
        {
            ssd1306InvertArea(0, y - 1, 120, 10);
            ssd1306InvertArea(0, y, 120, 10);
            ssd1306Refresh();
            //HAL_Delay(1);
        }
    }
    else
    {
        for ( ; y >= max_y; y--)
        {
            ssd1306InvertArea(0, y + 1, 120, 10);
            ssd1306InvertArea(0, y, 120, 10);
            ssd1306Refresh();
            //HAL_Delay(1);
        }
    }
}
void affiche_menu(menuItem menu,int line)
{
	ssd1306ClearScreen();
	ssd1306DrawString(0,0,menu.name,&Font_5x8);
	ssd1306DrawLine(0,8,128,8);
	for (int i=0;i<MAX_LINE_SCREEN;i++)
	{
		if(menu.line[i].name!=null)
			ssd1306DrawString(0,10*i+10,menu.line[line+i].name,&Font_5x8);
			switch (menu.line[line+i].type)
			{
				case 'b':
					if(*((bool*)menu.line[i+line].param)==true)
						ssd1306DrawString(90,10*i+10,"yes",&Font_5x8);
					else
						ssd1306DrawString(90,10*i+10,"no",&Font_5x8);
					break;
				case 'i':
					ssd1306PrintInt(90,10*i+10," ",*((unsigned int*)menu.line[i+line].param),&Font_3x6);
					break;
				case 'l':
					ssd1306PrintInt(90,10*i+10," ",*((unsigned long*)menu.line[i+line].param),&Font_3x6);
					break;
				case 'f':
					ssd1306DrawString(110,i*10+10,"->",&Font_3x6);
					break;
				case 'm':
					ssd1306DrawString(115,i*10+10,">",&Font_3x6);
					break;
			}
	}
	char nmbr_item=0;
	while(menu.line[nmbr_item].name!=null)
	{
		nmbr_item++;
	}
	if (nmbr_item>MAX_LINE_SCREEN)
	{
		int heightOneItem=54/nmbr_item;
		ssd1306DrawRect(123,heightOneItem*line+10,3,MAX_LINE_SCREEN*heightOneItem);
		ssd1306Refresh();
	}
}
int modify_bool_param( char *param_name, unsigned char *param)
{
    char str[4];
    bool param_copy = (bool)*param;

    hal_ui_clear_scr();

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
    	int joystick=Expander_Joy_State();
    	switch (joystick)
    	{
    		case LEFT :
    			// Wait until button is released
    			anti_rebonds();
    			return SUCCESS;
    			break;

    		case DOWN:
    		case UP :
    			anti_rebonds();
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
    			anti_rebonds();

    			*param = param_copy;
				hal_ui_clear_scr();
				ssd1306Refresh();
				return SUCCESS;
				break;
        }
    }
    return SUCCESS;
}


int modify_long_param( char *param_name,unsigned long *param)
{
	int step=1;
    char str[40];
    unsigned long param_copy = *param;
    char collone=0;
    hal_ui_clear_scr();

    // Write the parameter name
    ssd1306DrawString(0, 0,param_name, &Font_5x8);
    ssd1306DrawLine(0, 9, 128, 9);

    sprintf(str, "%10d", (int)param_copy);
    ssd1306DrawString(0, 28, str, &Font_8x8);
    ssd1306DrawString(0, 50, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
    ssd1306DrawString(0, 57, "      'LEFT'  TO RETURN.", &Font_3x6);
	ssd1306DrawString((10-collone)*8,20,"^",&Font_8x8);
	ssd1306DrawString((10-collone)*8,36,"v",&Font_8x8);
    ssd1306Refresh();

    while (1)
    {
        // Exit Button
    	int joystick=Expander_Joy_State();
    	switch (joystick)
    	{
    		case LEFT :
    			anti_rebonds();
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
    			anti_rebonds();
    			param_copy +=1;
    			//param_copy += (step*pow(10,collone));
				ssd1306ClearRect(0, 28, 164, 8);
				sprintf(str, "%10d", param_copy);
				ssd1306DrawString(0, 28, str, &Font_8x8);
				ssd1306Refresh();
				break;
    		case DOWN :
    			anti_rebonds();
				//param_copy -= (step*pow(10,collone));
				param_copy -= 1;
				ssd1306ClearRect(0, 28, 164, 8);
				sprintf(str, "%10i", (int)param_copy);
				ssd1306DrawString(0, 28, str, &Font_8x8);
				ssd1306Refresh();
				break;
    		case RIGHT :
    			anti_rebonds();
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
