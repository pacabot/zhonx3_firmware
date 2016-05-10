/*
 * robotInterface.h
 *
 *  Created on: 4 juin 2015
 *      Author: zhonx
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

//#define DEBUG_ROBOT_INTERFACE

#include "middleware/wall_sensors/wall_sensors.h"

void goOrientation(char *orientationZhonx, char directionToGo);
void doUTurn(positionRobot *positionZhonx);
void moveZhonxArc(int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char end_mid_of_case, char chain);
int floorSensorCalibrate(void);
int waitValidation(unsigned long timeout);
void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx);
void move_zhonx(int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case,
                    char chain);
walls getCellState();
walls ask_cell_state ();
void print_cell_state (walls cell_state);
void waitStart();
int saveMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate);
int loadMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate);

#endif /* ROBOTINTERFACE_H_ */
