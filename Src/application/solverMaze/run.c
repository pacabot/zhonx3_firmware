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
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/moves/basicMoves/basicMoves.h"

/* peripherale inlcudes*/
#include "peripherals/motors/motors.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/tone/tone.h"
#endif

int run(labyrinthe *maze, positionRobot *positionZhonx, coordinate start_coordinate, coordinate end_coordinate, int runType)
{
    int rv = RUN_E_SUCCESS;
    unsigned int runTime;
    coordinate way[(MAZE_SIZE * MAZE_SIZE)];
    int max_speed_rotation, max_speed_translation, min_speed_translation;

    if (runType == 1)
    {
        max_speed_rotation = zhonxSettings.speeds_run1.max_speed_rotation;
        max_speed_translation = zhonxSettings.speeds_run1.max_speed_traslation;
        min_speed_translation = zhonxSettings.speeds_run1.min_speed;
    }
    else if (runType == 2)
    {
        max_speed_rotation = zhonxSettings.speeds_run2.max_speed_rotation;
        max_speed_translation = zhonxSettings.speeds_run2.max_speed_traslation;
        min_speed_translation = zhonxSettings.speeds_run2.min_speed;
    }
    else
    {
        return RUN_E_ERROR;
    }

    memset(&way, 0, sizeof(way));
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);

    clearMazelength(maze);
    computeCellWeight(maze, end_coordinate, FALSE, FALSE);
    rv = moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);
    if (rv != MAZE_SOLVER_E_SUCCESS)
    {
        ssd1306PrintfAtLine(64, 2, &Font_5x8,"the shorter way");
        ssd1306PrintfAtLine(64, 3, &Font_5x8,"is not find");
        HAL_Delay(1000);
        #ifdef PRINT_BLUETOOTH_BASIC_DEGUG // todo debug
        bluetoothWaitReady();
        bluetoothPrintf("no solution");
        #endif
        return MAZE_SOLVER_E_ERROR;
    }

    telemetersStart();
    ssd1306PrintfAtLine(55, 0, &Font_3x6, "WAIT START...");
    ssd1306Refresh();
    waitStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "SCAN...");
    ssd1306Refresh();

    runTime = HAL_GetTick();
    rv = moveRealZhonxArc(maze, positionZhonx, way, max_speed_rotation, max_speed_translation, min_speed_translation);
    if (rv != MAZE_SOLVER_E_SUCCESS)
    {
        ssd1306PrintfAtLine(60,3, &Font_5x8, "error way");
        ssd1306Refresh();
    }
    runTime = HAL_GetTick() - runTime;

    basicMoveStop();
    motorsDriverSleep(ON); //because flash write cause interrupts damages
    telemetersStop();//because flash write cause interrupts damages

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(0, 0, &Font_5x8, "TARGET REACHED !!!");
    ssd1306PrintfAtLine(0, 2, &Font_7x8, "TIME : %d.%ds", runTime / 1000, runTime % 1000);
    ssd1306Refresh();
    tone(G5, 100);
    HAL_Delay(100);
    tone(A5, 100);
    HAL_Delay(100);
    tone(B5, 50);
    HAL_Delay(50);
    tone(B5, 400);
    HAL_Delay(2000);

    if(zhonxSettings.return_to_start_cell == true)
    {
        telemetersStart();
        goToPosition(maze, positionZhonx, start_coordinate);
        doUTurn(positionZhonx, SAFE_SPEED_ROTATION, SAFE_SPEED_TRANSLATION, SAFE_SPEED_TRANSLATION);
    }
    telemetersStop();

#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothWaitReady();
    bluetoothPrintf("TIME : %d.%ds", runTime / 1000, runTime % 1000);
#endif
    ssd1306PrintfAtLine(30, 4, &Font_5x8, "END RUN");
    ssd1306Refresh();
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
    return rv;
}
