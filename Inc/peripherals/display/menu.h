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


typedef struct
{
char *name;
char type;
int (*param)(void);
}lineItem;

typedef struct
{
		char *name;
		lineItem line[20];
}menuItem;

int menu(menuItem);
void menuHighlightedMove(unsigned char y, unsigned char max_y);
void displayMenu(menuItem menu,int first_line);
int modifyBoolParam( char *param_name, unsigned char *param);
int modifyLongParam( char *param_name,long *param);

#endif /* MENU_H_ */
