/*
 * run.c
 *
 *  Created on: 4 juin 2015
 *      Author: Colin
 */

#include "stm32f4xx_hal.h"
#include "config/basetypes.h"

/* peripherale inlcudes*/
#include "peripherals/motors/motors.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
/* meddleware include */
#include "middleware/settings/settings.h"
/*application include */
#include "application/solverMaze/solverMaze.h"
#include "application/solverMaze/robotInterface.h"
#include "application/solverMaze/run.h"

void run1(labyrinthe *maze, positionRobot *positionZhonx, char posXStart,
		char posYStart)
{
	char choice;
	do
	{
		coordinate way = { 0, 0, 0, 0};
		choice = -1;
		waitStart ();
		// go to the center position
		motorsSleepDriver (OFF);
		telemetersStart();
		clearMazelength (maze);
		poids (maze, zhonxSettings.x_finish_maze, zhonxSettings.y_finish_maze, false);
		printLength(*maze);
		moveVirtualZhonx (*maze, *positionZhonx, &way, zhonxSettings.x_finish_maze, zhonxSettings.y_finish_maze);
		moveRealZhonxArc (maze, positionZhonx, way.next);//, &zhonxSettings.x_finish_maze, &zhonxSettings.y_finish_maze);

//		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		HAL_Delay (2000);
		exploration (maze, positionZhonx, posXStart, posYStart);
		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		doUTurn (positionZhonx);

		ssd1306ClearScreen ();
		ssd1306DrawString (10, 10, "presse \"RIGHT\" to ", &Font_5x8);
		ssd1306DrawString (10, 18, "do a new run 1", &Font_5x8);
		ssd1306Refresh ();
		while (choice == -1)
		{
			if (expanderJoyFiltered () == JOY_RIGHT)
			{
				choice = 1;
			}

			if (expanderJoyFiltered () != JOY_RIGHT
					&& expanderJoyFiltered () != 0)
			{
				choice = 0;
			}
		}
	}while (choice == 1);
}

void run2(labyrinthe *maze, positionRobot *positionZhonx, char posXStart,
		char posYStart)
{
	coordinate way; // = {0,0,NULL);
	char choice;
	do
	{
		choice = -1;
		moveVirtualZhonx (*maze, *positionZhonx, &way,
				zhonxSettings.x_finish_maze, zhonxSettings.y_finish_maze);
		waitStart ();
		moveRealZhonxArc (maze, positionZhonx, way.next);
//		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		HAL_Delay (2000);
		exploration (maze, positionZhonx, posXStart, posYStart);
//		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		doUTurn (positionZhonx);
		ssd1306ClearScreen ();
		ssd1306DrawString (10, 10, "presse \"RIGHT\" to ", &Font_5x8);
		ssd1306DrawString (10, 18, "do a new run 2", &Font_5x8);
		ssd1306Refresh ();
		while (choice == -1)
		{
			if (expanderJoyFiltered () == JOY_RIGHT)
			{
				choice = 1;
			}

			if (expanderJoyFiltered () != JOY_RIGHT
					&& expanderJoyFiltered () != 0)
			{
				choice = 0;
			}
		}
	}while (choice == 1);
}
