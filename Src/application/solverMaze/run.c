/*
 * run.c
 *
 *  Created on: 4 juin 2015
 *      Author: Colin
 */

#include "config/basetypes.h"

#include "middleware/settings/settings.h"

#ifndef codeblocks
#include "stm32f4xx_hal.h"

/*application include */
#include "application/solverMaze/solverMaze.h"
#include "application/solverMaze/robotInterface.h"
#include "application/solverMaze/run.h"

/* Middleware declarations */
#include "middleware/controls/mazeControl/wallFollowControl.h"
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/controls/mazeControl/reposition.h"
#include "middleware/controls/mazeControl/spyPost.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/wall_sensors/wall_sensors.h"

/* peripherale inlcudes*/
#include "peripherals/motors/motors.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/tone/tone.h"
#endif

void run1(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate)
{
    char choice;
    coordinate way[MAZE_SIZE * MAZE_SIZE];
    memset(&way, 0, sizeof(way));
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);
    do
    {
        choice = -1;
        clearMazelength(maze);
        computeCellWeight(maze, end_coordinate, FALSE, FALSE);
        moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);

        telemetersStart();
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(30, 0, &Font_5x8, "WAIT START...");
        ssd1306Refresh();
        waitStart();
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(40, 0, &Font_5x8, "SCAN...");
        ssd1306Refresh();

        moveRealZhonxArc(&maze, positionZhonx, way);

        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(0, 0, &Font_5x8, "TARGET REACHED !!!");
        ssd1306Refresh();
        tone(G5, 100);
        HAL_Delay(100);
        tone(A5, 100);
        HAL_Delay(100);
        tone(B5, 50);
        HAL_Delay(50);
        tone(B5, 400);
        HAL_Delay(2000);

        goToPosition(maze, positionZhonx, start_oordinate);
//		if(zhonxSettings.calibration_enabled == TRUE)
//			calibrateSimple();
        doUTurn(positionZhonx);

        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(10, 0, "press \"RIGHT\" to ", &Font_5x8);
        ssd1306DrawStringAtLine(10, 1, "do a new run 1", &Font_5x8);
        ssd1306Refresh();
        while (choice == -1)
        {
            if (expanderJoyFiltered() == JOY_RIGHT)
            {
                choice = 1;
            }

            if (expanderJoyFiltered() != JOY_RIGHT && expanderJoyFiltered() != 0)
            {
                choice = 0;
            }
        }
    }
    while (choice == 1);
}

void run2(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate)
{
    coordinate way[MAZE_SIZE * MAZE_SIZE];
    char choice;
    do
    {
        choice = -1;
        clearMazelength(maze);
        computeCellWeight(maze, zhonxSettings.maze_end_coordinate, TRUE, FALSE);
        printMaze(*maze, positionZhonx->coordinate_robot);
        waitStart();
        moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);
        moveRealZhonxArc(maze, positionZhonx, way);
//		if(zhonxSettings.calibration_enabled == TRUE)
//			calibrateSimple();
        HAL_Delay(2000);
        goToPosition(maze, positionZhonx, start_oordinate);
//		if(zhonxSettings.calibration_enabled == TRUE)
//			calibrateSimple();
        doUTurn(positionZhonx);
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(10, 0, "press \"RIGHT\" to ", &Font_5x8);
        ssd1306DrawStringAtLine(10, 1, "do a new run 2", &Font_5x8);
        ssd1306Refresh();
        while (choice == -1)
        {
            if (expanderJoyFiltered() == JOY_RIGHT)
            {
                choice = 1;
            }

            if (expanderJoyFiltered() != JOY_RIGHT && expanderJoyFiltered() != 0)
            {
                choice = 0;
            }
        }
    }
    while (choice == 1);
}
