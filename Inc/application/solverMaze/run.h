/*
 * run.h
 *
 *  Created on: 4 juin 2015
 *      Author: Colin
 */

#ifndef RUN_H_
#define RUN_H_

/* Error codes */
#define RUN_E_SUCCESS  0
#define RUN_E_ERROR    MAKE_ERROR(RUN_MODULE_ID, 1)

int run(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate, int runType);

#endif /* RUN_H_ */
