/*
 * robotInterface.h
 *
 *  Created on: 4 juin 2015
 *      Author: zhonx
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

//#define DEBUG_ROBOT_INTERFACE

#include <application/solverMaze/robotInterface.h>
#include <application/solverMaze/solverMaze.h>
#include <middleware/settings/settings.h>
#include <middleware/wall_sensors/wall_sensors.h>

#define WAIT_START_DISTANCE 100.00

void goOrientation(char *orientationZhonx, char directionToGo, int max_speed_rotation);
void doUTurn(positionRobot *positionZhonx, int max_speed_rotation, int max_speed_translation, int min_speed_translation);
void moveZhonxArc(int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char end_mid_of_case, char chain);
int floorSensorCalibrate(void);
int waitValidation(unsigned long timeout);
void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx);
void move_zhonx(char direction_to_go, positionRobot *positionZhonx, unsigned int numberOfCell, char end_mid_of_case,
                char chain, int max_speed_rotation, int max_speed_translation, int min_speed_translation);
walls getCellState();
walls ask_cell_state ();
void print_cell_state (walls cell_state);
void waitStart();
int saveMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate);
int loadMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate);

#endif /* ROBOTINTERFACE_H_ */
