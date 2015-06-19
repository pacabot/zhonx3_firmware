/*
 * robotInterface.h
 *
 *  Created on: 4 juin 2015
 *      Author: zhonx
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

void goOrientation(char *orientationZhonx, char directionToGo);
void doUTurn(positionRobot *positionZhonx);
void moveZhonxArc (int direction_to_go, positionRobot *positionZhonx, int numberOfCase, char end_mid_of_case, char chain);
int floorSensorCalibrate(void);
int waitValidation(unsigned long timeout);
void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx);


#endif /* ROBOTINTERFACE_H_ */
