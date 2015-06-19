/*
 * run.c
 *
 *  Created on: 4 juin 2015
 *      Author: Colin
 */

#include "peripherals/display/smallfonts.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "middleware/settings/settings.h"
#include "application/solverMaze/solverMaze.h"
#include "stm32f4xx_hal.h"
#include "config/basetypes.h"

void run1(labyrinthe *maze, positionRobot *positionZhonx, char posXStart,
		char posYStart)
{
	char choice;
	do
	{
		choice = -1;
		waitStart ();
		exploration (maze, positionZhonx, zhonxSettings.x_finish_maze,
				zhonxSettings.y_finish_maze);
		if (zhonxSettings.calibration_enabled == true)
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
		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		HAL_Delay (2000);
		exploration (maze, positionZhonx, posXStart, posYStart);
		if (zhonxSettings.calibration_enabled == true)
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
