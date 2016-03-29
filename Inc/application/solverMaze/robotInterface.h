/*
 * robotInterface.h
 *
 *  Created on: 4 juin 2015
 *      Author: zhonx
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#include "middleware/wall_sensors/wall_sensors.h"

void goOrientation(char *orientationZhonx, char directionToGo);
void doUTurn(positionRobot *positionZhonx);
void moveZhonxArc (int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char end_mid_of_case, char chain);
int waitValidation(unsigned long timeout);
void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx);
walls ask_cell_state ();
void move_zhonx_arc (int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case, char chain);
walls getCellState ();


#endif /* ROBOTINTERFACE_H_ */
