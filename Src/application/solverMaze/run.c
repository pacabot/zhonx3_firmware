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

    int max_speed_rotation = RUN1_SPEED_ROTATION;
    int max_speed_translation = RUN1_MAX_SPEED_TRANSLATION;
    int min_speed_translation = RUN1_MIN_SPEED_TRANSLATION;

    memset(&way, 0, sizeof(way));
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);

    choice = -1;
    clearMazelength(maze);
    computeCellWeight(maze, end_coordinate, FALSE, FALSE);
    moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);

    telemetersStart();
    ssd1306PrintfAtLine(55, 0, &Font_3x6, "WAIT START...");
    ssd1306Refresh();
    waitStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "SCAN...");
    ssd1306Refresh();

    moveRealZhonxArc((labyrinthe *)&maze, positionZhonx, way, max_speed_rotation, max_speed_translation, min_speed_translation);

    moveEmergencyStop();
    motorsDriverSleep(ON); //because flash write cause interrupts damages
    telemetersStop();//because flash write cause interrupts damages

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
#ifdef RETURN_START_CELL
    telemetersStart();
    goToPosition(maze, positionZhonx, start_oordinate);
    //		if(zhonxSettings.calibration_enabled == TRUE)
    //			calibrateSimple();
    doUTurn(positionZhonx, SAFE_SPEED_ROTATION, SAFE_SPEED_TRANSLATION, SAFE_SPEED_TRANSLATION);
#endif
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothPrintf("uturn do\ngo back");
#endif
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "END RUN");
    ssd1306Refresh();
    telemetersStop();
    motorsDriverSleep(ON);
    HAL_Delay(1000);
}

void run2(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_oordinate, coordinate end_coordinate)
{
    coordinate way[MAZE_SIZE * MAZE_SIZE];
    char choice;

    int max_speed_rotation = RUN2_SPEED_ROTATION;
    int max_speed_translation = RUN2_MAX_SPEED_TRANSLATION;
    int min_speed_translation = RUN2_MIN_SPEED_TRANSLATION;

    memset(&way, 0, sizeof(way));
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);

    choice = -1;
    clearMazelength(maze);
    computeCellWeight(maze, end_coordinate, FALSE, FALSE);
    moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);

    telemetersStart();
    ssd1306PrintfAtLine(55, 0, &Font_3x6, "WAIT START...");
    ssd1306Refresh();
    waitStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "SCAN...");
    ssd1306Refresh();

    moveRealZhonxArc((labyrinthe *)&maze, positionZhonx, way, max_speed_rotation, max_speed_translation, min_speed_translation);

    moveEmergencyStop();
    motorsDriverSleep(ON); //because flash write cause interrupts damages
    telemetersStop();//because flash write cause interrupts damages

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
#ifdef RETURN_START_CELL
    telemetersStart();
    goToPosition(maze, positionZhonx, start_oordinate);
    //      if(zhonxSettings.calibration_enabled == TRUE)
    //          calibrateSimple();
    doUTurn(positionZhonx, SAFE_SPEED_ROTATION, SAFE_SPEED_TRANSLATION, SAFE_SPEED_TRANSLATION);
#endif
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothPrintf("uturn do\ngo back");
#endif
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "END RUN");
    ssd1306Refresh();
    telemetersStop();
    motorsDriverSleep(ON);
    HAL_Delay(1000);
}
