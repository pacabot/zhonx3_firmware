/*
 * menu.h
 *
 *  Created on: 4 déc. 2014
 *      Author: colin
 */

#ifndef MENU_H_
#define MENU_H_

#define MAX_LINE_SCREEN 	5

#define HIGHLIGHT_HEIGHT 	10
#define HIGHLIGHT_LENGHT 	120
#define ROW_HEIGHT			10
#define MARGIN				10

#define MAX_LINE_IN_MENU	20

typedef struct
{
    char *name;
    char type;
    int (*param)(void);
} lineItem;

typedef struct
{
    char *name;
    lineItem line[MAX_LINE_IN_MENU];
} menuItem;

int menu(const menuItem);
void menuHighlightedMove(unsigned char y, unsigned char max_y);
void displayMenu(const menuItem menu, int first_line);
int modifyBoolParam(char *param_name, unsigned char *param);
int modifyLongParam(char *param_name, long *param);
int modifyPresetParam(char *param_name, void *param);
void graphMotorSettings(float *acceleration, float *maxSpeed, float *deceleration);
void printGraphMotor(float acceleration, float maxSpeed, float deceleration);
void welcomeDisplay();
void killOnLowBattery();
void powerOffConfirmation();

#endif /* MENU_H_ */

