#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "config/basetypes.h"

/* peripherale inlcudes*/
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"

/* meddleware include */
#include "middleware/settings/settings.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/wallFollowControl.h"
#include "middleware/controls/motionControl/mainControl.h"

/*application include */
#include "application/solverMaze/solverMaze.h"
#include "application/solverMaze/robotInterface.h"
#include "application/solverMaze/run.h"

int maze(void)
{
	char posXStart, posYStart; // it's the coordinates which Zhonx have at the start
	labyrinthe maze;
	mazeInit (&maze);
	positionRobot positionZhonx;

	telemetersInit();
	telemetersStart();
	mainControlInit ();

	control_params.follow_state = true;
	follow_control.follow_type = FOLLOW_WALL;//NOFOLLOW;
	move(0, 0, 0, 0);
	HAL_Delay(500);

	/*init for different micromouse competition*/

	if (zhonxSettings.color_sensor_enabled == true) // if it's the Nimes micromouse competition
	{
		positionZhonx.x = MAZE_SIZE / 2;
		positionZhonx.y = MAZE_SIZE / 2; // the robot start at the middle of the maze
		positionZhonx.orientation = NORTH; // the robot is pointing the north
		zhonxSettings.x_finish_maze = 0;
		zhonxSettings.y_finish_maze = 0; // we want to go to the case how have address (0,0)
	}
	else // it's the Birmingham competition
	{
		positionZhonx.x = 0;
		positionZhonx.y = 0; // the robot start in the corner
		positionZhonx.orientation = EAST;
		// the position of the finish is defined in the menu
	}
	/*end of initialization for different micromouse competition*/
	positionZhonx.midOfCell = true;
	posXStart = positionZhonx.x;
	posYStart = positionZhonx.y;
	printMaze (maze, positionZhonx.x, positionZhonx.y);
//	if (zhonxSettings.calibration_enabled == true)
//	{
//		calibrateSimple ();
//	}
	motorsSleepDriver(OFF);
	for (int i = 0; i < 4; ++i)
	{
		rotate90WithCal(CW, 300, 0);
		while(isEndMove() != true);
		positionZhonx.orientation=(positionZhonx.orientation+1)%4;
		newCell (getCellState (), &maze, positionZhonx);
	}
	move (0, -CELL_LENGTH/2, 50, 0);
	while(isEndMove() != true);
	motorsSleepDriver(ON);

	printMaze(maze,positionZhonx.x, positionZhonx.y);
	do
	{
		waitStart ();
		exploration (&maze, &positionZhonx, zhonxSettings.x_finish_maze,
				zhonxSettings.y_finish_maze);
//		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		HAL_Delay (2000);
		exploration (&maze, &positionZhonx, posXStart, posYStart);
//		if (zhonxSettings.calibration_enabled == true)
//			calibrateSimple ();
		doUTurn (&positionZhonx);
		HAL_Delay (2000);
	}while (false
			== miniWayFind (&maze, posXStart, posYStart,
					zhonxSettings.x_finish_maze, zhonxSettings.y_finish_maze));
	run1 (&maze, &positionZhonx, posXStart, posYStart);
	run2 (&maze, &positionZhonx, posXStart, posYStart);
	return MAZE_SOLVER_E_SUCCESS;
}

void exploration(labyrinthe *maze, positionRobot* positionZhonx, char xFinish,
		char yFinish)
{
	coordinate way = { 0, 0, 0, 0};
	motorsSleepDriver (OFF);
	telemetersStart();
	HAL_Delay(1000);
	newCell (getCellState(), maze, *positionZhonx);
	telemetersStart();

	while (positionZhonx->x != xFinish || positionZhonx->y != yFinish)
	{
		clearMazelength (maze);
		poids (maze, xFinish, yFinish, true);
		printLength(*maze, positionZhonx->x, positionZhonx->y);
		moveVirtualZhonx (*maze, *positionZhonx, &way, xFinish, yFinish);
		moveRealZhonxArc (maze, positionZhonx, way.next);//, &xFinish, &yFinish);
	}
	telemetersStop();
	HAL_Delay (200);
	motorsSleepDriver (ON);

}



void moveVirtualZhonx(labyrinthe maze, positionRobot positionZhonxVirtuel,
		coordinate *way, char xFinish, char yFinish)
{
	while (positionZhonxVirtuel.x != xFinish
			|| positionZhonxVirtuel.y != yFinish)
	{
		printMaze (maze, positionZhonxVirtuel.x, positionZhonxVirtuel.y);
		if (maze.cell[(int) (positionZhonxVirtuel.x + 1)][(int) (positionZhonxVirtuel.y)].length + 1 == maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y)].length && positionZhonxVirtuel.x+1<MAZE_SIZE && maze.cell[(int)(positionZhonxVirtuel.x)][(int)(positionZhonxVirtuel.y)].wall_east==NO_WALL)
		{
			positionZhonxVirtuel.x = positionZhonxVirtuel.x + 1;
		}
		else if (maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y + 1)].length + 1 == maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y)].length && positionZhonxVirtuel.y+1<MAZE_SIZE && maze.cell[(int)(positionZhonxVirtuel.x)][(int)(positionZhonxVirtuel.y)].wall_south==NO_WALL)
		{
			positionZhonxVirtuel.y = positionZhonxVirtuel.y + 1;
		}
		else if (maze.cell[(int) (positionZhonxVirtuel.x - 1)][(int) (positionZhonxVirtuel.y)].length + 1 == maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y)].length && positionZhonxVirtuel.x>0 && maze.cell[(int)(positionZhonxVirtuel.x)][(int)(positionZhonxVirtuel.y)].wall_west==NO_WALL)
		{
			positionZhonxVirtuel.x = positionZhonxVirtuel.x - 1;
		}
		else if (maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y - 1)].length + 1 == maze.cell[(int) (positionZhonxVirtuel.x)][(int) (positionZhonxVirtuel.y)].length && positionZhonxVirtuel.y>0 && maze.cell[(int)(positionZhonxVirtuel.x)][(int)(positionZhonxVirtuel.y)].wall_north==NO_WALL)
		{
			positionZhonxVirtuel.y = positionZhonxVirtuel.y - 1;
		}
		else
		{
			if (way!=NULL && way->previous != NULL)
				return;
			else
			{
				printMaze(maze,positionZhonxVirtuel.x,positionZhonxVirtuel.y);
				ssd1306DrawString (60, 0, "no solution", &Font_5x8);
				ssd1306Refresh ();
				motorsSleepDriver (ON);
				while (true)
				{
					HAL_Delay (200);
				}
			}
		}
		newDot (&way, positionZhonxVirtuel.x, positionZhonxVirtuel.y);
	}
	return;
}


void moveRealZhonxArc(labyrinthe *maze, positionRobot *positionZhonx, coordinate *way)
{
	walls cell_state;
	char endMidCase;
	char chain;
	coordinate *oldDote;
	int length;
	char additionY = 0;
	char additionX = 0;
	char orientaionToGo = NORTH;
	while (way != NULL)
	{
		length = 0;
		if (way->x == (positionZhonx->x + 1) && way->y == positionZhonx->y)
		{
			additionX = 1;
			additionY = 0;
			orientaionToGo = EAST;
		}
		else if (way->x == (positionZhonx->x - 1) && way->y == positionZhonx->y)
		{
			additionX = -1;
			additionY = 0;
			orientaionToGo = WEST;
		}
		else if (way->y == (positionZhonx->y - 1) && way->x == positionZhonx->x)
		{

			additionX = 0;
			additionY = -1;
			orientaionToGo = NORTH;
		}
		else if (way->y == (positionZhonx->y + 1) && way->x == positionZhonx->x)
		{

			additionX = 0;
			additionY = 1;
			orientaionToGo = SOUTH;
		}
		else
		{
			bluetoothPrintf("Error way : position zhonx x= %d y=%d \t way x= %d y=%d \n",positionZhonx->x,positionZhonx->y, way->x, way->y);
			HAL_Delay (200);
			motorsSleepDriver (ON);
			ssd1306DrawString (60, 0, "Error way", &Font_5x8);
			ssd1306Refresh ();
			while (1)
			{
				HAL_Delay (500);
			}
		}

		while ((way != NULL) && way->y == (positionZhonx->y + additionY)
				&& way->x == positionZhonx->x + additionX)
		{
			length++;
			positionZhonx->x = way->x;
			positionZhonx->y = way->y;
			oldDote = way;
			way = way->next;
			free (oldDote);
			if(way == NULL)
			{
				break;
			}
		}
		if ((positionZhonx->x != 8 || positionZhonx->y != 8) )
			endMidCase = false;
		else
			endMidCase = true;
		if (way == NULL)
			chain = false;
		else
			chain = true;
		move_zhonx_arc (orientaionToGo, positionZhonx, length, endMidCase, chain);
		cell_state = getCellState ();
		newCell (cell_state, maze, *positionZhonx);

	}
}



void poids(labyrinthe *maze, int xFinish, int yfinish, char wallNoKnow)
{
	int length = 0;
	int x = xFinish;
	int y = yfinish;
	maze->cell[x][y].length = length;
	coordinate *dotes_to_verifie = NULL;
	newDot (&dotes_to_verifie, x, y);
	coordinate *new_dotes_to_verifie = NULL;
	coordinate *pt = NULL;

	while (dotes_to_verifie != NULL)
	{
		length++;
		while (dotes_to_verifie != NULL)
		{
			x = dotes_to_verifie->x;
			y = dotes_to_verifie->y;
			pt = dotes_to_verifie->previous;
			free (dotes_to_verifie);
			dotes_to_verifie = pt;
			if ((maze->cell[x][y].wall_north == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_north == NO_KNOWN))
					&& maze->cell[x][y - 1].length > length - 1 && y > 0)
			{
				newDot (&new_dotes_to_verifie, x, y - 1);
				maze->cell[x][y - 1].length = length;
			}
			if ((maze->cell[x][y].wall_east == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_east == NO_KNOWN))
					&& maze->cell[x + 1][y].length > length&& x+1<MAZE_SIZE)
			{
				newDot (&new_dotes_to_verifie, x + 1, y);
				maze->cell[x + 1][y].length = length;
			}
			if ((maze->cell[x][y].wall_south == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_south == NO_KNOWN))
					&& maze->cell[x][y + 1].length > length&& y+1<MAZE_SIZE)
			{
				newDot (&new_dotes_to_verifie, x, y + 1);
				maze->cell[x][y + 1].length = length;
			}
			if ((maze->cell[x][y].wall_west == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_west == NO_KNOWN))
					&& maze->cell[x - 1][y].length > length && x > 0)
			{
				newDot (&new_dotes_to_verifie, x - 1, y);
				maze->cell[x - 1][y].length = length;
			}
		}
		dotes_to_verifie = new_dotes_to_verifie;
		new_dotes_to_verifie = NULL;
	}
}

void newDot(coordinate **old_dot, int x, int y)
{
	if (*old_dot != NULL)
	{
		(*old_dot)->next = (coordinate*)calloc_s (1, sizeof(coordinate));
		coordinate *pt = *old_dot;
		*old_dot = (*old_dot)->next;
		(*old_dot)->previous = pt;
	}
	else
	{
		(*old_dot) = (coordinate*) calloc_s (1, sizeof(coordinate));
	}
	(*old_dot)->x = x;
	(*old_dot)->y = y;
}
void mazeInit(labyrinthe *maze)
{
#ifndef test
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		for (int y = 0; y < MAZE_SIZE; y++)
		{
			maze->cell[i][y].wall_north = NO_KNOWN;
			maze->cell[i][y].wall_west = NO_KNOWN;
			maze->cell[i][y].wall_south = NO_KNOWN;
			maze->cell[i][y].wall_east = NO_KNOWN;
			maze->cell[i][y].length = 2000;
		}
	}
	for (int i = 0; i < 16; i++)
	{
		maze->cell[i][0].wall_north = WALL_PRESENCE;
		maze->cell[i][MAZE_SIZE - 1].wall_south = WALL_PRESENCE;
		maze->cell[0][i].wall_west = WALL_PRESENCE;
		maze->cell[MAZE_SIZE - 1][i].wall_east = WALL_PRESENCE;
	}
	newCell((walls){WALL_PRESENCE, WALL_PRESENCE, WALL_PRESENCE},maze, (positionRobot){8,8,SOUTH,false});
#else
	labyrinthe maze_initial=
	{
		{
			{
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,NO_WALL,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,WALL_PRESENCE,NO_WALL,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,WALL_PRESENCE,NO_WALL,NO_WALL,2000}},
			{
				{	WALL_PRESENCE,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,NO_WALL,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,NO_WALL,WALL_PRESENCE,WALL_PRESENCE,2000},
				{	NO_WALL,WALL_PRESENCE,WALL_PRESENCE,NO_WALL,2000}}}};

	*maze=maze_initial;
#endif
}

void printMaze(const labyrinthe maze, const int x_robot, const int y_robot)
{
#ifdef DEBUG
	ssd1306ClearRect(0,0,64,64);
	int size_cell_on_oled = ((63) / MAZE_SIZE);
	int x, y;
	for (y = 0; y < MAZE_SIZE; y++)
	{
		for (x = 0; x < MAZE_SIZE; x++)
		{
			if (maze.cell[x][y].wall_north == WALL_PRESENCE)
			{
				ssd1306DrawLine (x * size_cell_on_oled, y * size_cell_on_oled,
		x * size_cell_on_oled + size_cell_on_oled + 1, y * size_cell_on_oled);
			}
			else if (maze.cell[x][y].wall_north == NO_KNOWN)
			{
				ssd1306DrawDashedLine (x * size_cell_on_oled, y * size_cell_on_oled,
			   x * size_cell_on_oled + size_cell_on_oled + 1, y * size_cell_on_oled);
			}
			if (maze.cell[x][y].wall_west == WALL_PRESENCE)
			{
				ssd1306DrawLine (x * size_cell_on_oled, y * size_cell_on_oled,
								 x * size_cell_on_oled, y * size_cell_on_oled + size_cell_on_oled + 1);
			}
			else if (maze.cell[x][y].wall_west == NO_KNOWN)
			{
				ssd1306DrawDashedLine (x * size_cell_on_oled, y * size_cell_on_oled,
									   x * size_cell_on_oled, y * size_cell_on_oled + size_cell_on_oled + 1);
			}

			if (maze.cell[x][y].wall_south == WALL_PRESENCE)
			{
				ssd1306DrawLine (x * size_cell_on_oled,(y + 1) * size_cell_on_oled,
			 size_cell_on_oled + x * size_cell_on_oled,(y + 1) * size_cell_on_oled);
			}
			else if (maze.cell[x][y].wall_south == NO_KNOWN)
			{
				ssd1306DrawDashedLine (x * size_cell_on_oled,(y + 1) * size_cell_on_oled,
				   size_cell_on_oled + x * size_cell_on_oled,(y + 1) * size_cell_on_oled);
			}
			if (maze.cell[x][y].wall_east == WALL_PRESENCE)
			{
				ssd1306DrawLine ((x + 1) * size_cell_on_oled, y * size_cell_on_oled,
								 (x + 1) * size_cell_on_oled, y * size_cell_on_oled + size_cell_on_oled + 1);
			}
			else if (maze.cell[x][y].wall_east == NO_KNOWN)
			{
				ssd1306DrawDashedLine  ((x + 1) * size_cell_on_oled, y * size_cell_on_oled,
										(x + 1) * size_cell_on_oled, y * size_cell_on_oled + size_cell_on_oled + 1);
			}
		}
	}
	printLength(maze, x_robot, y_robot);
	ssd1306DrawRect((x_robot * size_cell_on_oled) +1, (y_robot * size_cell_on_oled)+1, 2, 2);
	ssd1306Refresh ();
#endif
}

void* calloc_s(size_t nombre, size_t taille)
{
	void* pt = calloc (nombre, taille);
	if (pt == NULL)
	{
		printf ("null pointer exception, full memory");
		while (1)
		{
			HAL_Delay(500);
		}
	}
	return pt;
}

void printLength(const labyrinthe maze,const int x_robot, const int y_robot)
{
#if DEBUG > 2
    bluetoothPrintf ("zhonx : %d; %d\n", x_robot, y_robot);
	bluetoothPrintf ("  ");
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		bluetoothPrintf ("%5d", i);
	}
	bluetoothPrintf ("\n\n");
	for (int i = 0; i < MAZE_SIZE; i++)
	{
		bluetoothPrintf ("%2d ", i);
		for (int j = 0; j < MAZE_SIZE; j++)
		{
			if (maze.cell[j][i].wall_north == NO_KNOWN)
			{
				bluetoothPrintf("====*");
			}
			else
			{
				bluetoothPrintf ("    *");
			}
		}
		bluetoothPrintf ("\n   ");
		for (int j = 0; j < MAZE_SIZE; j++)
		{
			bluetoothPrintf ("%4d", maze.cell[j][i].length);
			if (maze.cell[j][i].wall_east == NO_KNOWN)
			{
				bluetoothPrintf ("|");
			}
			else
			{
				bluetoothPrintf (" ");
			}
		}
		bluetoothPrintf ("\n");
	}
	bluetoothPrintf ("\n");
#endif
}

void clearMazelength(labyrinthe* maze)
{
	int x, y;
	for (y = 0; y < MAZE_SIZE; y++)
	{
		for (x = 0; x < MAZE_SIZE; x++)
		{
			maze->cell[x][y].length = 2000;
		}
	}
}

char miniWayFind(labyrinthe *maze, char xStart, char yStart, char xFinish,
		char yFinish)
{
	// TODO not find the shorter in distance way but the faster
	coordinate *way1 = NULL;
	coordinate *way2 = NULL;
	clearMazelength (maze);
	poids (maze, xFinish, yFinish, true);
	positionRobot position;
	position.midOfCell = true;
	position.x = xStart;
	position.y = yStart;
	position.orientation = NORTH;
	moveVirtualZhonx (*maze, position, way1, xFinish, yFinish);
	clearMazelength (maze);
	poids (maze, xFinish, yFinish, false);
	moveVirtualZhonx (*maze, position, way2, xFinish, yFinish);
	ssd1306ClearScreen ();
	char waySame = diffWay (way1, way2);
	switch (waySame)
	{
		case true :
			ssd1306DrawString (0, 20, "2 way = : yes", &Font_5x8);
			break;
		case false :
			ssd1306DrawString (0, 20, "2 way = : no", &Font_5x8);
			break;
	}
	deleteWay (way1);
	deleteWay (way2);
	ssd1306Refresh ();
	HAL_Delay (3000);
	return (waySame);
}

char diffWay(coordinate *way1, coordinate *way2)
{
	while (way1 != NULL && way2 != NULL)
	{
		if (way1->x != way2->x || way1->y != way2->y)
		{
			return false;
		}
		way1 = way1->next;
		way2 = way2->next;
	}
	if (way1 != NULL || way2 != NULL)
	{
		return false;
	}
	return true;
}

void deleteWay(coordinate *way) // TODO: verify the function
{
	while (way != NULL)
	{
		way = way->next;
		free (way->previous);
	}
}

void waitStart()
{
	ssd1306ClearRect(SSD1306_LCDWIDTH/2,0,SSD1306_LCDWIDTH/2,SSD1306_LCDHEIGHT);
	ssd1306Printf(SSD1306_LCDWIDTH/2,0,&Font_5x8,"wait start");
	ssd1306Refresh();
	while (expanderJoyFiltered() != JOY_RIGHT)
	{
		HAL_Delay (20);
	}
	ssd1306ClearRect(SSD1306_LCDWIDTH/2,0,SSD1306_LCDWIDTH/2,SSD1306_LCDHEIGHT);
	ssd1306Refresh();
//TODO : wait start with front sensors
//	unsigned char sensors_state = hal_sensor_get_state(app_context.sensors);
//	while(check_bit(sensors_state, SENSOR_F10_POS)==true)
//		sensors_state = hal_sensor_get_state(app_context.sensors);
//	HAL_Delay(200);
//	while(check_bit(sensors_state, SENSOR_F10_POS)==false)
//		sensors_state = hal_sensor_get_state(app_context.sensors);
}
