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
extern void eepromTest();
extern void encoderTest();
extern void adxrs620Test();
extern void mulimeterTest();
extern void telemetersTest();
extern void toneTest();
extern void motorsTest();
extern void lineSensorsTest();
extern void lineFollower();
extern int lineSensorsCalibration(void);
extern void maze();
extern void testWallsSensors();
extern void movesTest(void);
extern void rotateTest(void);
extern void expenderLedTest();
extern int wallSensorsCalibrationFront(void);
extern int wallSensorsCalibrationDiag(void);
extern void testFlash(void);
extern int setMeddle(void);
extern int setDark(void);
extern int pidCalculator(void);
extern void telemetersGetCalibrationValues(void);
extern void spyPostCalibration(void);
extern void spyPostTest();
extern int spyPostReadCalibration();
extern int _Factor;
extern int _KP;
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

float toto = 4.0;
float titi = 4.0;
float tata = 4.0;

const menuItem testGraphicMenu =
{
		"GRAPHICS",
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
		"MAZE",
		{
				{"new maze",'f',		(void*)maze},
				{"calibration",'b',		(void*)&zhonxSettings.calibration_enabled},
				{"color finish",'b',	(void*)&zhonxSettings.nime_competition},
				{"x finish",'i',		(void*)&zhonxSettings.maze_end_coordinate.x},
				{"y finish",'i',		(void*)&zhonxSettings.maze_end_coordinate.y}
		}

};

const menuItem follower_menu=
{
		"LINE FOLL",
		{
				{"line follower",'f',	(void*)lineFollower},
				{"calibration",'f',		(void*)lineSensorsCalibration},
				//{"Sensor Bluetooth",'f', (void*)lineSensorSendBluetooth},
				{"Set F. K",'i', (void*)&_Factor},
				{"Set F. KP",'i', (void*)&_KP},
		}
};

const menuItem parameters_menu=
{
		"PARAMS",
		{
				{"calibration front",'f',(void*)wallSensorsCalibrationFront},
				{"calibration diag",'f',(void*)wallSensorsCalibrationDiag},
				{"Send calib values", 'f', (void *)telemetersGetCalibrationValues},
				{"Set BT baudrate", 'p', (void *)&BTpresetBaudRate}
		}
};
const menuItem peripheral_test_menu=
{
		"PERIPH.",
		{0}
};
const menuItem tests_menu=
{
		"TESTS",
		{
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
		"CONTROL",
		{
				{"follow the wall",'f',		(void*)movesTest},
				{"rotate",'f',				(void*)rotateTest},
				{"PID calculator",'f',		(void*)pidCalculator},
				{"spy post Cal.",'f',		(void*)spyPostCalibration},
		}
};

const menuItem zhonxNameMenu =
{
		"SET MAME",
		{
				{"Meddle", 'f', setMeddle},
				{"Dark", 'f', setDark},
				{NULL, 0, NULL}
		}
};

const menuItem mainMenu =
{
		CONFIG_ZHONX_INFO_ADDR,
		{
				//	{"telemeters calibration",'f',		(void*)telemeterFrontCalibration},
				{"Maze menu",'m',			(void*)&maze_menu},
				{"Unit tests",'m',			(void*)&tests_menu},
				{"Control menu",'m',		(void*)&control_menu},
				{"Line menu",'m', 			(void*)&follower_menu},
				{"Parameters menu",'m',		(void*)&parameters_menu},
				{"Test graph",'m',			(void*)&testGraphicMenu},
				{"Test flash",'f',          (void*)&testFlash},
				{"Zhonx Name",'m',          (void*)&zhonxNameMenu},
				{0,0,0}
		}
};

extern I2C_HandleTypeDef hi2c1;

int menu(const menuItem Menu)
{
    signed char line_screen = 1;
    signed char line_menu = 0;
    // Display main menu
    displayMenu(Menu, line_menu);
    ssd1306InvertArea(0, MARGIN, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
    ssd1306Refresh();
    while (true)
    {
        HAL_Delay(1);
        int joystick = expanderJoyFiltered();
        //		killOnLowBattery();
        switch (joystick)
        {
            case JOY_LEFT:
                return SUCCESS;
                // Joystick down
            case JOY_DOWN:
                toneItMode(100, 10);
                if (Menu.line[line_menu + 1].name != null)
                {
                    line_menu++;
                    if (line_screen >= MAX_LINE_SCREEN)
                    {
                        displayMenu(Menu, line_menu - (line_screen - 1));
                        ssd1306InvertArea(0, line_screen * MARGIN,
                        HIGHLIGHT_LENGHT,
                                          HIGHLIGHT_HEIGHT);
                        ssd1306Refresh();
                    }
                    else
                    {
                        line_screen++;
                        menuHighlightedMove((line_screen - 1) * ROW_HEIGHT + 1, (line_screen) * ROW_HEIGHT);
                    }
                }
                else
                {
                    line_menu = 0;
                    line_screen = 1;
                    displayMenu(Menu, line_menu - (line_screen - 1));
                    ssd1306InvertArea(0, line_screen * MARGIN,
                    HIGHLIGHT_LENGHT,
                                      HIGHLIGHT_HEIGHT);
                    ssd1306Refresh();
                }
                break;
            case JOY_UP:
                toneItMode(100, 10);
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
                        menuHighlightedMove((line_screen + 1) * ROW_HEIGHT - 1, (line_screen) * ROW_HEIGHT);
                    }
                }
                else
                {
                    while (Menu.line[line_menu + 1].name != null)
                    {
                        line_menu++;
                    }
                    if (line_menu < MAX_LINE_SCREEN - 1)
                    {
                        displayMenu(Menu, 0);
                        line_screen = line_menu + 1;
                    }
                    else
                    {
                        line_screen = MAX_LINE_SCREEN;
                        displayMenu(Menu, line_menu - (MAX_LINE_SCREEN - 1));
                    }
                    ssd1306InvertArea(0, MARGIN * line_screen, HIGHLIGHT_LENGHT,
                    HIGHLIGHT_HEIGHT);
                    ssd1306Refresh();
                }
                break;
            case JOY_RIGHT: // Validate button joystick right
                toneItMode(8000, 20);
                switch (Menu.line[line_menu].type)
                {
                    case 'b':
                        modifyBoolParam(Menu.line[line_menu].name, (unsigned char*) Menu.line[line_menu].param);
                        break;
                    case 'i':
                        modifyLongParam(Menu.line[line_menu].name, (long*) (int*) Menu.line[line_menu].param);
                        break;
                    case 'l':
                        modifyLongParam(Menu.line[line_menu].name, (long*) Menu.line[line_menu].param);
                        break;
                    case 'm':
                        menu(*(const menuItem*) Menu.line[line_menu].param);
                        break;
                    case 'f':
                        if (Menu.line[line_menu].param != NULL)
                        {
                            ssd1306ClearScreen(MAIN_AREA);
                            ssd1306Refresh();
                            Menu.line[line_menu].param();
                        }
                        break;
                    case 'g':
                        graphMotorSettings((float*) Menu.line[line_menu - 3].param,
                                           (float*) Menu.line[line_menu - 2].param,
                                           (float*) Menu.line[line_menu - 1].param);
                        break;
                    case 'p':
                        modifyPresetParam(Menu.line[line_menu].name, Menu.line[line_menu].param);
                        break;
                    default:
                        break;
                }
                HAL_Delay(50);
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
        for (; y <= max_y; y++)
        {
            ssd1306InvertArea(0, y - 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
            ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
            if (y % 2) //refresh if pair, increases the refresh speed
                ssd1306Refresh();
            while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
                ;
        }
    }
    else
    {
        //		ssd1306InvertArea(0, y-HIGHLIGHT_HEIGHT+1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT*2);
        for (; y >= max_y; y--)
        {
            ssd1306InvertArea(0, y + 1, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
            ssd1306InvertArea(0, y, HIGHLIGHT_LENGHT, HIGHLIGHT_HEIGHT);
            if (y % 2) //refresh if pair, increases the refresh speed
                ssd1306Refresh();
            while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
                ;
        }
    }
    ssd1306Refresh();
}

void displayMenu(const menuItem menu, int line)
{
    //char str[5];
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306ClearRect(0, 0, 38, 7); //clear title name
    ssd1306DrawString(0, -1, menu.name, &Font_3x6);
    for (int i = 0; i < MAX_LINE_SCREEN; i++)
    {
        if (menu.line[i].name != null)
            ssd1306DrawStringAtLine(0, i, menu.line[line + i].name, &Font_5x8);
        switch (menu.line[line + i].type)
        {
            case 'b':
                if (*((bool*) menu.line[i + line].param) == true)
                    ssd1306DrawStringAtLine(90, i, "yes", &Font_5x8);
                else
                    ssd1306DrawStringAtLine(90, i, "no", &Font_5x8);
                break;
            case 'i':
                ssd1306PrintIntAtLine(90, i, " ", *((unsigned int*) menu.line[i + line].param), &Font_3x6);
                break;
            case 'l':
                ssd1306PrintIntAtLine(90, i, " ", *((unsigned long*) menu.line[i + line].param), &Font_3x6);
                break;
            case 'f':
                ssd1306DrawStringAtLine(110, i, ">", &Font_3x6);
                break;
            case 'm':
                ssd1306DrawStringAtLine(110, i, "->", &Font_3x6);
                break;
            case 'p':
                ssd1306PrintIntAtLine(90, i, " ", (long) ((presetParam*) menu.line[i + line].param)->p_value,
                                      &Font_3x6);
                break;
        }
    }
    uint8_t nmbr_item = 0;
    while (menu.line[nmbr_item].name != null)
    {
        nmbr_item++;
    }
    if (nmbr_item > MAX_LINE_SCREEN)
    {
        //int heightOneItem=54/nmbr_item;
        ssd1306DrawRect(123, (54 * line) / nmbr_item + MARGIN, 3, (54 * MAX_LINE_SCREEN) / nmbr_item);
    }
}

int modifyBoolParam(char *param_name, unsigned char *param)
{
    char str[4];
    bool param_copy = (bool) *param;

    ssd1306ClearScreen(MAIN_AREA);

    // Write the parameter name
    ssd1306DrawStringAtLine(0, 0, param_name, &Font_5x8);

    if (param_copy == true)
    {
        sprintf(str, "YES");
    }
    else
    {
        sprintf(str, "NO");
    }
    ssd1306DrawStringAtLine(0, 2, str, &Font_8x8);
    ssd1306DrawStringAtLine(0, 3, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
    ssd1306DrawStringAtLine(0, 4, "      'LEFT'  TO RETURN", &Font_3x6);
    ssd1306Refresh();

    while (1)
    {
        int joystick = expanderJoyFiltered();
        HAL_Delay(100);
        switch (joystick)
        {
            case JOY_LEFT:
                return SUCCESS;
                break;

            case JOY_DOWN:
            case JOY_UP:
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
                ssd1306ClearRectAtLine(0, 2, 128);
                ssd1306DrawStringAtLine(0, 2, str, &Font_8x8);
                ssd1306Refresh();
                break;

            case JOY_RIGHT:

                *param = param_copy;
                ssd1306ClearScreen(MAIN_AREA);
                ssd1306Refresh();
                return SUCCESS;
                break;
        }
    }
    return SUCCESS;
}

int modifyLongParam(char *param_name, long *param)
{
    int step = 1;
    char str[40];
    long param_copy = *param;
    char collone = 0;

    ssd1306ClearScreen(MAIN_AREA);

    // Write the parameter name
    ssd1306ClearRect(0, 0, 38, 7); //clear title name
    ssd1306DrawString(0, -1, param_name, &Font_3x6);

    sprintf(str, "%10i", (int) param_copy);

    ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
    ssd1306DrawStringAtLine(0, 3, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
    ssd1306DrawStringAtLine(0, 4, "      'LEFT'  TO RETURN", &Font_3x6);

    ssd1306DrawStringAtLine((10 - collone) * 8, 0, "-", &Font_8x8);
    ssd1306DrawStringAtLine((10 - collone) * 8, 2, "-", &Font_8x8);
    ssd1306Refresh();

    while (1)
    {
        // Exit Button
        int joystick = expanderJoyFiltered();
        switch (joystick)
        {
            case JOY_LEFT:
                if (collone == 10)
                    return SUCCESS;
                else
                {
                    collone++;
                    ssd1306ClearRectAtLine(0, 0, 128);
                    ssd1306ClearRectAtLine(0, 2, 128);
                    ssd1306DrawStringAtLine((9 - collone) * 9, 0, "-", &Font_8x8);
                    ssd1306DrawStringAtLine((9 - collone) * 9, 2, "-", &Font_8x8);
                    ssd1306Refresh();
                }
                break;
            case JOY_UP:

                //param_copy +=1;
                param_copy += (step * pow(10, collone));
                sprintf(str, "%10i", (int) param_copy);
                ssd1306ClearRectAtLine(0, 1, 128);
                ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
                ssd1306Refresh();
                break;
            case JOY_DOWN:

                param_copy -= (step * pow(10, collone));
                //param_copy -= 1;
                sprintf(str, "%10i", (int) param_copy);
                ssd1306ClearRectAtLine(0, 1, 128);
                ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
                ssd1306Refresh();
                break;
            case JOY_RIGHT:
                if (collone == 0)
                {
                    *param = param_copy;
                    ssd1306Refresh();
                    return SUCCESS;
                }
                else
                {
                    collone--;
                    ssd1306ClearRectAtLine(0, 0, 128);
                    ssd1306ClearRectAtLine(0, 2, 128);
                    ssd1306DrawStringAtLine((9 - collone) * 9, 0, "-", &Font_8x8);
                    ssd1306DrawStringAtLine((9 - collone) * 9, 2, "-", &Font_8x8);
                    ssd1306Refresh();
                }
                break;
            default:
                break;
        }
    }

    return SUCCESS;
}

int modifyPresetParam(char *param_name, presetParam *param)
{
    char str[40];
    presetParam *preset = param;
    int param_copy = 0;
    int preset_val = 0;
    int *p_preset_val;
    int presetBufferLen = 0;
    int *p_presetBuffer = (int *) (preset->presetBuffer);
    int rv;

    preset_val = *(p_presetBuffer);

    // Check if the current value is present into the preset buffer
    while (preset_val != -0x7FFFFFFF)
    {
        if (preset_val == *((int *) preset->p_value))
        {
            // Current value has been found into preset buffer
            param_copy = preset_val;
            // Keep a pointer toward the value in preset buffer
            p_preset_val = p_presetBuffer;
            // Don't break here, in order to count until end of the buffer
        }
        p_presetBuffer++;
        presetBufferLen++;
        preset_val = *p_presetBuffer;
    }

    // Decrement presetBufferLen by 1, as the last element is not used
    presetBufferLen--;

    if (presetBufferLen <= 0)
    {
        // This should never happen!
        return ERROR;
    }

    if (param_copy == 0)
    {
        // Current value is not into preset buffer
        param_copy = *((int *) preset->p_value);
        p_presetBuffer = (int *) (preset->presetBuffer);
        *((int *) (preset->p_value)) = *p_presetBuffer;
    }
    else
    {
        // Set back the stored pointer
        p_presetBuffer = p_preset_val;
    }

    ssd1306ClearScreen(MAIN_AREA);

    // Write the parameter name
    ssd1306ClearRect(0, 0, 38, 7); //clear title name
    ssd1306DrawString(0, -1, param_name, &Font_3x6);

    // Write parameter's current value
    sprintf(str, "%10i", param_copy);

    ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
    ssd1306DrawStringAtLine(0, 3, "PRESS 'RIGHT' TO VALIDATE", &Font_3x6);
    ssd1306DrawStringAtLine(0, 4, "      'LEFT'  TO RETURN", &Font_3x6);

    ssd1306DrawStringAtLine(5 * 8, 0, "-", &Font_8x8);
    ssd1306DrawStringAtLine(5 * 8, 2, "-", &Font_8x8);

    ssd1306Refresh();

    while (1)
    {
        int joystick = expanderJoyFiltered();

        switch (joystick)
        {
            case JOY_LEFT:
                // Exit button
                return SUCCESS;

            case JOY_UP:
                p_presetBuffer++;
                if (*p_presetBuffer == -0x7FFFFFFF)
                {
                    // Reset pointer to the first element of preset buffer
                    p_presetBuffer = (int *) (preset->presetBuffer);
                }
                param_copy = *p_presetBuffer;
                sprintf(str, "%10i", (int) param_copy);
                ssd1306ClearRectAtLine(0, 1, 128);
                ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
                ssd1306Refresh();
                break;

            case JOY_DOWN:
                p_presetBuffer--;
                if (p_presetBuffer == (int *) (preset->presetBuffer))
                {
                    // Set pointer to the last element of the preset buffer
                    p_presetBuffer = &(((int *) preset->presetBuffer)[presetBufferLen]);
                }
                param_copy = *(p_presetBuffer);
                sprintf(str, "%10i", (int) param_copy);
                ssd1306ClearRectAtLine(0, 1, 128);
                ssd1306DrawStringAtLine(0, 1, str, &Font_8x8);
                ssd1306Refresh();
                break;

            case JOY_RIGHT:
                // Validate button
                *((int *) (preset->p_value)) = param_copy;
                if (preset->callback != NULL)
                {
                    // Call the callback function
                    preset->callback(param_copy, NULL);
                    // TODO: Check returned value if needed
                }
                ssd1306Refresh();
                return SUCCESS;

            default:
                break;
        }
    }

    return SUCCESS;
}

void graphMotorSettings(float *acceleration, float *maxSpeed, float *deceleration)
{
    int number_value = 0;
    float* values[3] = { acceleration, maxSpeed, deceleration };
    while (true)
    {
        printGraphMotor(*acceleration, *maxSpeed, *deceleration);
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
                *(values[number_value]) -= 0.1;
                break;
            case JOY_UP:
                *(values[number_value]) += 0.1;
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
void printGraphMotor(float acceleration, float maxSpeed, float deceleration)
{
    char str[10];
    ssd1306ClearScreen(MAIN_AREA);
    char point1[2] = { (char) ((0 - maxSpeed) / (0 - acceleration)), 64 - (char) maxSpeed };
    char point2[2] = { (char) (128 - (0 - maxSpeed) / (0 - deceleration)), 64 - (char) (maxSpeed) };
    ssd1306DrawLine(0, 64, point1[0], point1[1]);
    ssd1306DrawLine(point1[0], point1[1], point2[0], point2[1]);
    ssd1306DrawLine(point2[0], point1[1], 128, 64);

    sprintf(str, "%.2fM.S^2", acceleration);
    ssd1306DrawString((0 + point1[0]) / 2 + 2, (64 + point1[1]) / 2 + 2, str, &Font_3x6);

    sprintf(str, "%.2fM.S", maxSpeed);
    ssd1306DrawString((point1[0] + point2[0]) / 2, (point1[1] + point2[1]) / 2, str, &Font_3x6);

    sprintf(str, "%.2fM.S^2", deceleration);
    ssd1306DrawString((point2[0] + 128) / 2 - 27, (point2[1] + 64) / 2, str, &Font_3x6);
    ssd1306Refresh();
}
void welcomeDisplay()
{

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawBmp(Pacabot_bmp, 1, 1, 128, 40);
    ssd1306Refresh();
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
        ;

    for (int i = 0; i <= 100; i += 1)
    {
        ssd1306ProgressBar(10, 35, i);
        ssd1306Refresh();
        HAL_Delay(5);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawBmp(five_years, 1, 1, 128, 54);
    ssd1306Refresh();
}

void powerOffConfirmation()
{
    unsigned char confirm = FALSE;
    modifyBoolParam("TURN POWER OFF ?", &confirm);
    if (confirm == TRUE)
    {
        modifyBoolParam("SAVE PARAM ?", &confirm);
        if (confirm == TRUE)
        {
            //			save_setting();
        }
        halt();
        while (1)
            ;
    }
}
void killOnLowBattery()
{
    //	if(multimeterGetBatVoltage() < (BATTERY_LOWER_VOLTAGE_NO_LOAD)*1000)
    //	{
    //		tone(A2,500);
    //		HAL_Delay(400);
    //		halt();
    //	}
}
