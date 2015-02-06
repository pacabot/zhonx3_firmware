/*
 * menu_colin.h
 *
 *  Created on: 4 déc. 2014
 *      Author: colin
 */

#ifndef MENU_COLIN_H_
#define MENU_COLIN_H_

#define MAX_LINE_SCREEN 5
typedef struct {
char *name;
char type;
int (*param)(void);
}lineItem;

typedef struct{
		char *name;
		lineItem line[20];
}menuItem;
int menu(menuItem);
void menu_animate(unsigned char y, unsigned char max_y);
void affiche_menu(menuItem menu,int first_line);
int modify_bool_param( char *param_name, unsigned char *param);
int modify_long_param( char *param_name,long *param);

#endif /* MENU_COLIN_H_ */
