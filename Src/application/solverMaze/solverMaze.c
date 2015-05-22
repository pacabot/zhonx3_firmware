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

/* meddleware include */
#include "middleware/settings/settings.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/controls/motionControl/positionControl.h"
#include "middleware/controls/motionControl/speedControl.h"
#include "middleware/controls/motionControl/transfertFunction.h"
#include "middleware/controls/motionControl/followControl.h"
#include "middleware/controls/motionControl/mainControl.h"

/*application include */
#include "application/solverMaze/solverMaze.h"
int MAX_SPEED_TRANSLATION = 400;

void test_maze()
{
//	telemetersInit();
//	telemetersStart();
//	mainControlInit ();
//
//	control_params.follow_state = TRUE;
//	follow_control.follow_type = FOLLOW_WALL;//NOFOLLOW
//	//position_control.position_type = GYRO; unused
//	move(0, 0, 0, 0);
//	HAL_Delay(500);
//	positionRobot position_zhonx={8,8,NORTH,true};
//	coordinate *end_way=null;
//	coordinate *start_way;
//	new_dot(&end_way,8,7);
//	start_way=end_way;
//	new_dot(&end_way,8,6);
//	new_dot(&end_way,8,5);
//	new_dot(&end_way,8,4);
//	new_dot(&end_way,7,4);
//	new_dot(&end_way,8,4);
//	new_dot(&end_way,8,3);
//	labyrinthe maze;
//	maze_init(&maze);
//	moveRealZhonxArc(&maze,&position_zhonx,start_way);
}

int maze(void)
{
	char posXStart, posYStart; // it's the coordinates which Zhonx have at the start
	labyrinthe maze;
	maze_init (&maze);
	positionRobot positionZhonx;

	telemetersInit();
	telemetersStart();
	mainControlInit ();

	control_params.follow_state = TRUE;
	follow_control.follow_type = NOFOLLOW;//FOLLOW_WALL;
	move(0, 0, 0, 0);
	HAL_Delay(500);

	/*init for different micromouse competition*/

	if (zhonxSettings.color_sensor_enabled == true) // if it's the Nimes micromouse competition
	{
		positionZhonx.x = MAZE_SIZE / 2;
		positionZhonx.y = MAZE_SIZE / 2; // the robot start at the middle of the maze
		positionZhonx.orientation = NORTH; // the robot is pointing the north
//		zhonxSettings.x_finish_maze = 0;
//		zhonxSettings.y_finish_maze = 0; // we want to go to the case how have address (0,0)
	}
	else
	{
		positionZhonx.x = MAZE_SIZE / 2;
		positionZhonx.y = MAZE_SIZE / 2; // the robot start at the middle of the maze
		positionZhonx.orientation = NORTH; // the robot is pointing the northzhonxSettings.x_finish_maze = 0;
		zhonxSettings.y_finish_maze = positionZhonx.y + zhonxSettings.y_finish_maze;
		zhonxSettings.x_finish_maze = positionZhonx.x + zhonxSettings.x_finish_maze;

	}
//	else // it's the Birmingham competition
//	{
//		positionZhonx.x = 0;
//		positionZhonx.y = 0; // the robot start in the corner
//		positionZhonx.orientation = EAST;
//		// the position of the finish is defined in the menu
//	}
	/*end of initialization for different micromouse competition*/
	positionZhonx.midOfCell = true;
	posXStart = positionZhonx.x;
	posYStart = positionZhonx.y;
	print_maze (maze, positionZhonx.x, positionZhonx.y);
	if (zhonxSettings.calibration_enabled == true)
	{
		calibrateSimple ();
	}
//	for (int i = 0; i < 4; ++i)
//	{
//		rotate90WithCal(CW, 300, 0);
//		while(isEndMove() != TRUE);
//		positionZhonx.orientation=(positionZhonx.orientation+1)%4;
//		new_cell (getCellState (), &maze, positionZhonx);
		print_maze(maze,positionZhonx.x, positionZhonx.y);
//	}
	do
	{
		waitStart ();
		exploration (&maze, &positionZhonx, zhonxSettings.x_finish_maze,
				zhonxSettings.y_finish_maze);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
		HAL_Delay (2000);
		exploration (&maze, &positionZhonx, posXStart, posYStart);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
		doUTurn (&positionZhonx);
		HAL_Delay (2000);
	}while (false
			== mini_way_find (&maze, posXStart, posYStart,
					zhonxSettings.x_finish_maze, zhonxSettings.y_finish_maze));
	waitStart ();
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
	HAL_Delay(500);
	new_cell (getCellState(), maze, *positionZhonx);
	telemetersStart();

	while (positionZhonx->x != xFinish || positionZhonx->y != yFinish)
	{
		clearMazelength (maze);
		poids (maze, xFinish, yFinish, true);
		print_length(*maze);
		moveVirtualZhonx (*maze, *positionZhonx, &way, xFinish, yFinish);
		moveRealZhonxArc (maze, positionZhonx, way.next);//, &xFinish, &yFinish);
	}
	telemetersStop();
	HAL_Delay (200);
	motorsSleepDriver (ON);

}

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
			calibrateSimple ();
		HAL_Delay (2000);
		exploration (maze, positionZhonx, posXStart, posYStart);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
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
			calibrateSimple ();
		HAL_Delay (2000);
		exploration (maze, positionZhonx, posXStart, posYStart);
		if (zhonxSettings.calibration_enabled == true)
			calibrateSimple ();
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

void moveVirtualZhonx(labyrinthe maze, positionRobot positionZhonxVirtuel,
		coordinate *way, char xFinish, char yFinish)
{
//	telemetersStop();
//	motorsSleepDriver(ON);
	while (positionZhonxVirtuel.x != xFinish
			|| positionZhonxVirtuel.y != yFinish)
	{
//		clearMazelength (&maze);
		//poids (&maze, xFinish, yFinish, true);
		print_maze (maze, positionZhonxVirtuel.x, positionZhonxVirtuel.y);
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
			if (way->previous != null)
				return;
			else
			{
				char boucle = true;
				print_maze(maze,positionZhonxVirtuel.x,positionZhonxVirtuel.y);
				ssd1306DrawString (60, 0, "no solution", &Font_5x8);
				ssd1306Refresh ();
				motorsSleepDriver (ON);
				while (boucle)
				{
				}
			}
		}
		new_dot (&way, positionZhonxVirtuel.x, positionZhonxVirtuel.y);
	}
	telemetersStart();
	motorsSleepDriver(OFF);
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
	while (way != null)
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
			bluetoothPrintf("position zhonx x= %d y=%d \t way x= %d y=%d \n",positionZhonx->x,positionZhonx->y, way->x, way->y);
			HAL_Delay (200);
			motorsSleepDriver (ON);
			ssd1306DrawString (60, 0, "Error way", &Font_5x8);
			ssd1306Refresh ();
			while (1)
				;
		}

		while (way->y == (positionZhonx->y + additionY)
				&& way->x == positionZhonx->x + additionX)
		{
			length++;
			positionZhonx->x = way->x;
			positionZhonx->y = way->y;
//			bluetoothPrintf(" position zhonx = way x= %d y=%d \n",positionZhonx->x,positionZhonx->y);
			oldDote = way;
			way = way->next;
			free (oldDote);
		}
		if ((positionZhonx->x != 8 || positionZhonx->y != 8) )
			endMidCase = false;
		else
			endMidCase = true;
		if (way == null)
			chain = false;
		else
			chain = true;
		move_zhonx_arc (orientaionToGo, positionZhonx, length, endMidCase, chain);
		cell_state = getCellState ();
		new_cell (cell_state, maze, *positionZhonx);

	}
}

void move_zhonx_arc (int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case, char chain)
{
	int speed_end;
	int turn = (4 + direction_to_go - positionZhonx->orientation) % 4;
	positionZhonx->orientation = direction_to_go;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
			if (positionZhonx->midOfCell == true)
			{
				while(isEndMove() != TRUE);
				move (90, 0, MAX_SPEED_ROTATION, 0);
			}
			else
			{
				moveRotateCW90(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
				numberOfCell --;

			}
			break;
		case UTURN :
			if(positionZhonx->midOfCell==false)
			{
				numberOfCell --;
			}
			moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
			break;
		case LEFT :
			if (positionZhonx->midOfCell == true)
			{
				while(isEndMove() != TRUE);
				move (-90, 0, MAX_SPEED_ROTATION, 0);
			}
			else
			{
				moveRotateCCW90(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
				numberOfCell --;
			}

			break;
	}
	if (positionZhonx->midOfCell == end_mid_of_case)
	{
		/*
		 * numberOfCell-=CELL_LENGTH/2;
		 * numberOfCell+=CELL_LENGTH/2;
		 */
	}
	else if (positionZhonx->midOfCell == true)
	{
		moveStartCell(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
		numberOfCell --;
	}
	else // so endMidOfCase=true and positionZhonx->midOfCase=false
	{
		moveHalfCell(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	}
	if (chain == true)
		speed_end = MAX_SPEED_ROTATION;
	else
	{
		speed_end = 0;
	}
	moveCell (numberOfCell, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	positionZhonx->midOfCell = end_mid_of_case;

}

void new_cell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx)
{
#ifdef debug
	/*print walls position*/
		ssd1306ClearRect(64,0,64,64);
		if (new_walls.front == WALL_PRESENCE)
		{
			ssd1306FillRect(64,49,54,5);
		}
		switch (new_walls.next_front)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(64,0,54,5);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(64,0,54,5);
				break;
			default:
				break;
		}
		switch (new_walls.left)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(64,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(64,0,5,54);
				break;
			default:
				break;
		}
		switch (new_walls.right)
		{
			case WALL_PRESENCE:
				ssd1306FillRect(113,0,5,54);
				break;
			case NO_KNOWN :
				ssd1306DrawRect(113,0,5,54);
				break;
			default:
				break;
		}
		ssd1306Refresh();
		/*end print wall position*/
#endif
//		telemetersStop();
//		motorsSleepDriver(ON);
	switch (positionZhonx.orientation)
	{
		case NORTH :
//				maze->cell[(int)(positionZhonx.x)][(int)(positionZhonx.y-1)].wall_north=new_walls.next_front;
//					maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y- 2)].wall_south = new_walls.next_front;
			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_east = new_walls.right;
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_west = new_walls.left;

				if (positionZhonx.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.x + 1)][(int) (positionZhonx.y)].wall_west = new_walls.right; // TODO : verify the "y-1"
				if (positionZhonx.x > 0)
					maze->cell[(int) (positionZhonx.x - 1)][(int) (positionZhonx.y)].wall_east = new_walls.left;
			}
			if (positionZhonx.y > 0)
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y - 1)].wall_south = new_walls.front;

			maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_north = new_walls.front;
			break;

		case EAST :

				if(positionZhonx.midOfCell == false)
				{
					maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_south = new_walls.right;
					maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_north = new_walls.left;

					if (positionZhonx.y < (MAZE_SIZE - 1))
						maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y + 1)].wall_north = new_walls.right;
					if (positionZhonx.y > 0)
						maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y - 1)].wall_south = new_walls.left;

				}
			if (positionZhonx.x < (MAZE_SIZE - 1) )
				maze->cell[(int) (positionZhonx.x + 1)][(int) (positionZhonx.y)].wall_west = new_walls.front;
			maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_east = new_walls.front;
			break;

		case SOUTH :

			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_west = new_walls.right;
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_east = new_walls.left;

				if (positionZhonx.x > 0)
					maze->cell[(int) (positionZhonx.x - 1)][(int) (positionZhonx.y)].wall_east = new_walls.right;
				if (positionZhonx.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.x + 1)][(int) (positionZhonx.y)].wall_west = new_walls.left;
			}
			if (positionZhonx.y > 0)
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y + 1)].wall_north = new_walls.front;
			maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_south =
					new_walls.front;
			break;

		case WEST :
			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_north = new_walls.right;
				maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_south = new_walls.left;


				if (positionZhonx.y > 0)
					maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y - 1)].wall_south = new_walls.right;
				if (positionZhonx.y < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y + 1)].wall_north = new_walls.left;
			}
			if (positionZhonx.x > 0)
				maze->cell[(int) (positionZhonx.x - 1)][(int) (positionZhonx.y)].wall_east = new_walls.front;
			maze->cell[(int) (positionZhonx.x)][(int) (positionZhonx.y)].wall_west = new_walls.front;
			break;
	}
	telemetersStart();
	motorsSleepDriver(OFF);
}

void poids(labyrinthe *maze, int xFinish, int yfinish, char wallNoKnow)
{
	int length = 0;
	int x = xFinish;
	int y = yfinish;
	maze->cell[x][y].length = length;
	coordinate *dotes_to_verifie = NULL;
	new_dot (&dotes_to_verifie, x, y);
	coordinate *new_dotes_to_verifie = NULL;
	coordinate *pt = NULL;

	while (dotes_to_verifie != NULL)
	{
		length++;
		while (dotes_to_verifie != NULL)
		{
			//printf("x: %2d y:%2d\n",x,y);
			//printf(" %d\n%d %d\n %d\n\n",maze->cell[x][y].wall_north,maze->cell[x][y].wall_west ,maze->cell[x][y].wall_east,maze->cell[x][y].wall_south);
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
				new_dot (&new_dotes_to_verifie, x, y - 1);
				maze->cell[x][y - 1].length = length;
			}
			if ((maze->cell[x][y].wall_east == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_east == NO_KNOWN))
					&& maze->cell[x + 1][y].length > length&& x+1<MAZE_SIZE)
			{
				new_dot (&new_dotes_to_verifie, x + 1, y);
				maze->cell[x + 1][y].length = length;
			}
			if ((maze->cell[x][y].wall_south == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_south == NO_KNOWN))
					&& maze->cell[x][y + 1].length > length&& y+1<MAZE_SIZE)
			{
				new_dot (&new_dotes_to_verifie, x, y + 1);
				maze->cell[x][y + 1].length = length;
			}
			if ((maze->cell[x][y].wall_west == NO_WALL
					|| (wallNoKnow == true
							&& maze->cell[x][y].wall_west == NO_KNOWN))
					&& maze->cell[x - 1][y].length > length && x > 0)
			{
				new_dot (&new_dotes_to_verifie, x - 1, y);
				maze->cell[x - 1][y].length = length;
			}
		}
		//print_length(*maze);
		dotes_to_verifie = new_dotes_to_verifie;
		new_dotes_to_verifie = NULL;
	}
}

void new_dot(coordinate **old_dot, int x, int y)
{
	//printf("x : %d ",x);
	//printf("y : %d ",y);
	if (*old_dot != NULL)
	{
		(*old_dot)->next = calloc_s (1, sizeof(coordinate));
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
//#define test
void maze_init(labyrinthe *maze)
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
	new_cell((walls){WALL_PRESENCE, WALL_PRESENCE, WALL_PRESENCE},maze, (positionRobot){8,8,SOUTH,FALSE});
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

void print_maze(const labyrinthe maze, const int x_robot, const int y_robot)
{
#ifdef debug
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
	//print_length(maze);
	ssd1306DrawPixel(x_robot * size_cell_on_oled+1, y_robot * size_cell_on_oled+1);
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
			;
	}
	return pt;
}

void print_length(const labyrinthe maze)
{
//	bluetoothPrintf ("  ");
//	for (int i = 0; i < MAZE_SIZE; i++)
//	{
//		bluetoothPrintf ("%4d", i);
//	}
//	bluetoothPrintf ("\n\n");
//	for (int i = 0; i < MAZE_SIZE; i++)
//	{
//		bluetoothPrintf ("%2d ", i);
//		for (int j = 0; j < MAZE_SIZE; j++)
//		{
//			if (maze.cell[j][i].wall_north == NO_KNOWN)
//			{
//				bluetoothPrintf("====*");
//			}
//			else
//			{
//				bluetoothPrintf ("    *");
//			}
//		}
//		bluetoothPrintf ("\n   ");
//		for (int j = 0; j < MAZE_SIZE; j++)
//		{
//			bluetoothPrintf ("%4d", maze.cell[j][i].length);
//			if (maze.cell[j][i].wall_east == NO_KNOWN)
//			{
//				bluetoothPrintf ("|");
//			}
//			else
//			{
//				bluetoothPrintf (" ");
//			}
//		}
//		bluetoothPrintf ("\n");
//	}
//	bluetoothPrintf ("\n");
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

char mini_way_find(labyrinthe *maze, char xStart, char yStart, char xFinish,
		char yFinish)
{
	// TODO trouver non pas le chemin le plus court mais le chemin le plus rapide
	coordinate *way1 = null;
	coordinate *way2 = null;
	clearMazelength (maze);
	poids (maze, xFinish, yFinish, true);
	moveVirtualZhonx (*maze, (positionRobot ) { true, xStart, yStart, NORTH },
			way1, xFinish, yFinish);
	clearMazelength (maze);
	poids (maze, xFinish, yFinish, false);
	moveVirtualZhonx (*maze, (positionRobot ) { true, xStart, yStart, NORTH },
			way2, xFinish, yFinish);
	//ssd1306ClearScreen ();
	char waySame = diffWay (way1, way2);
	switch (waySame)
	{
		case true :
			//ssd1306DrawString (0, 20, "2 way = : yes", &Font_5x8);
			break;
		case false :
			//ssd1306DrawString (0, 20, "2 way = : no", &Font_5x8);
			break;
	}
	deleteWay (way1);
	deleteWay (way2);
	//ssd1306Refresh ();
	HAL_Delay (3000);
	return (waySame);
}

char diffWay(coordinate *way1, coordinate *way2)
{
	while (way1 != null && way2 != null)
	{
		if (way1->x != way2->x || way1->y != way2->y)
		{
			return false;
		}
		way1 = way1->next;
		way2 = way2->next;
	}
	if (way1 != null || way2 != null)
	{
		return false;
	}
	return true;
}

void deleteWay(coordinate *way) // TODO: verify the function
{
	while (way != null)
	{
		way = way->next;
		free (way->previous);
	}
}

void waitStart()
{
	ssd1306ClearScreen();
	ssd1306Printf(0,0,&Font_5x8,"wait start");
	ssd1306Refresh();
	while (expanderJoyFiltered() != JOY_RIGHT)
		{
		;
		}
//	while (expanderJoyFiltered() != JOY_SEVERAL)
//		{
//		;
//		}
	ssd1306ClearScreen();
	ssd1306Refresh();
//TODO : wait start
//	unsigned char sensors_state = hal_sensor_get_state(app_context.sensors);
//	while(check_bit(sensors_state, SENSOR_F10_POS)==true)
//		sensors_state = hal_sensor_get_state(app_context.sensors);
//	HAL_Delay(200);
//	while(check_bit(sensors_state, SENSOR_F10_POS)==false)
//		sensors_state = hal_sensor_get_state(app_context.sensors);
}

void calibrateSimple()
{
//	motorsSleepDriver (OFF);
//	char orientation=0;
//	unsigned char sensors_state = 0;
//	for(int i=0; i<2;i++)
//	{
//		sensors_state =hal_sensor_get_state(app_context.sensors);
//		if (check_bit(sensors_state, SENSOR_L10_POS) == false)
//		{
//			goOrientation(&orientation,orientation-1);
//		}
//		else if (check_bit(sensors_state, SENSOR_R10_POS) == false)
//		{
//			goOrientation(&orientation,orientation+1);
//		}
// 		step_motors_basic_move(70);
//		HAL_Delay(500);
//		step_motors_basic_move(-((CELL_LENGTH/2)-46));
//	}
//	goOrientation(&orientation,0);
//	HAL_Delay(100);
//	motorsSleepDriver(ON);

}

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
			while(isEndMove() != TRUE);
			move (-90, 0, MAX_SPEED_ROTATION, 0);
			while(isEndMove() != TRUE);
			break;
		case UTURN :
			while(isEndMove() != TRUE);
			move (180, 0, MAX_SPEED_ROTATION, 0);
			while(isEndMove() != TRUE);
			break;
		case LEFT :
			while(isEndMove() != TRUE);
			move (90, 0, MAX_SPEED_ROTATION, 0);
			while(isEndMove() != TRUE);
			break;
	}
}

void doUTurn(positionRobot *positionZhonx)
{
	motorsSleepDriver(OFF);
	goOrientation (&positionZhonx->orientation,
			(positionZhonx->orientation + 2) % 4);

	move (0, -CELL_LENGTH/2, 50, 0);
	while (isEndMove() != TRUE);
	HAL_Delay(200);
	motorsSleepDriver(ON);
}

int sensor_calibrate(void)
{
//	int rv;
//	int i = 0;
//	unsigned long arrival_color = 30000;
//	unsigned long area_color = 500000;
//
//	lineSensorsInit ();
//	lineSensorsStart ();
//	while (1)
//	{
//		//ssd1306ClearScreen ();
//		//ssd1306Printf (0, 9, &Font_5x8, "Present arrival color");
//		//ssd1306Printf (0, 64 - 9, &Font_5x8, "'RIGHT' TO VALIDATE");
//		//ssd1306Refresh ();
//
//		arrival_color = lineSensors.front.adc_value;
//		//ssd1306Printf (10, 18, &Font_5x8, "Color sens: %i", arrival_color);
//
//		//ssd1306Refresh ();
//
//		rv = wait_validation (500);
//		if (rv == JOY_RIGHT)
//		{
//			// Value validated
//			for (i = 0; i < 100; i++)
//			{
//				arrival_color += lineSensors.front.adc_value;
//				HAL_Delay (50);
//			}
//			arrival_color /= i;
//			//ssd1306ClearScreen ();
//			//ssd1306Printf (2, 9, &Font_5x8, "Value %i validated",
//			//		arrival_color);
//			//ssd1306Refresh ();
//			HAL_Delay (1000);
//			break;
//		}
//		else if (rv == JOY_LEFT)
//		{
//			// User aborted
//			//ssd1306ClearScreen ();
//			//ssd1306Printf (2, 9, &Font_5x8, "Calibration aborted");
//			//ssd1306Refresh ();
//			HAL_Delay (1000);
//			return 0;
//		}
//	}
//
//	while (1)
//	{
//		//ssd1306ClearScreen ();
//		//ssd1306Printf (0, 9, &Font_5x8, "Present area color");
//		//ssd1306Printf (0, 64 - 9, &Font_5x8, "'RIGHT' TO VALIDATE");
//		////ssd1306Refresh ();
//
//		area_color = lineSensors.front.adc_value;
//		//ssd1306Printf (10, 18, &Font_5x8, "Color sens: %i", area_color);
//
//		//ssd1306Refresh ();
//
//		rv = wait_validation (500);
//		if (rv == JOY_RIGHT)
//		{
//			// Value validated
//			for (i = 0; i < 100; i++)
//			{
//				area_color += lineSensors.front.adc_value;
//				HAL_Delay (50);
//			}
//			area_color /= i;
//			//ssd1306ClearScreen ();
//			//ssd1306Printf (2, 9, &Font_5x8, "Value %i validated", area_color);
//			//ssd1306Refresh ();
//			HAL_Delay (1000);
//			break;
//		}
//	}
//
//	zhonxSettings.threshold_color = (MAX(arrival_color, area_color)
//			- MIN(arrival_color, area_color)) / 2;
//	//ssd1306ClearScreen ();
//	//ssd1306Printf (1, 1, &Font_5x8, "diff col : %d",
//	//		zhonxSettings.threshold_color);
//	//ssd1306Refresh ();
//	HAL_Delay (2000);
//	zhonxSettings.threshold_color += MIN(arrival_color, area_color);
//	zhonxSettings.threshold_greater = (arrival_color > area_color);
//
//	return 0;
}

int wait_validation(unsigned long timeout)
{
	timeout += HAL_GetTick ();
	do
	{
		switch (expanderJoyFiltered ())
		{
			case JOY_RIGHT :
				return JOY_RIGHT;
				break;
			case JOY_LEFT :
				return JOY_LEFT;
				break;
		}
	}while (timeout > HAL_GetTick ());
	return -1;
}
