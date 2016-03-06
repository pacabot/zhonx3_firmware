/*
 * @file : robotInterface.c
 * @author : Colin
 * @version : V 2.0
 * @date : 4 juin 2015
 * @brief : this file contain all the necessary stuff for make interface
 *          between the maze solver and the robot move function.
 */

#include "config/basetypes.h"
#include "config/config.h"
#include "stm32f4xx_hal.h"
/* peripherale inlcudes*/
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/telemeters/telemeters.h"

/* meddleware include */
#include "middleware/controls/motionControl/mainControl.h"

/*application include */
#include "application/solverMaze/solverMaze.h"

/* Middleware declarations */
#include "application/solverMaze/robotInterface.h"

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
			while(hasMoveEnded() != TRUE);
			move (-90, 0, MAX_SPEED_ROTATION, 0);
			while(hasMoveEnded() != TRUE);
			break;
		case UTURN :
			while(hasMoveEnded() != TRUE);
			move (180, 0, MAX_SPEED_ROTATION, 0);
			while(hasMoveEnded() != TRUE);
			break;
		case LEFT :
			while(hasMoveEnded() != TRUE);
			move (90, 0, MAX_SPEED_ROTATION, 0);
			while(hasMoveEnded() != TRUE);
			break;
	}
}

void move_zhonx_arc (int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case, char chain)
{
	int turn = (4 + direction_to_go - positionZhonx->orientation) % 4;
	positionZhonx->orientation = direction_to_go;
	switch (turn)
	{
		case FORWARD :
			break;
		case RIGHT :
			if (positionZhonx->midOfCell == true)
			{
				while(hasMoveEnded() != TRUE);				//todo rotate in place
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
				while(hasMoveEnded() != TRUE);
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
		moveHalfCell_IN(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	}
	moveCell(numberOfCell, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	positionZhonx->midOfCell = end_mid_of_case;

}

void doUTurn(positionRobot *positionZhonx)
{
	motorsDriverSleep(OFF);
	goOrientation (&positionZhonx->orientation,
			(positionZhonx->orientation + 2) % 4);

	moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	motorsDriverSleep(ON);
}

int floorSensorCalibrate(void)
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
	return 0;
}

int waitValidation(unsigned long timeout)
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

void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx)
{
#ifdef DEBUG
	/*print walls position*/
	static char i=1;
	i++;
	ssd1306ClearRect(64,0,64,64);
	if (new_walls.front == WALL_PRESENCE)
	{
		ssd1306FillRect(64,0,54,5);
	}
	if (new_walls.left == WALL_PRESENCE)
	{
		ssd1306FillRect(64,0,5,54);
	}
	if (new_walls.right == WALL_PRESENCE)
	{
		ssd1306FillRect(113,0,5,54);
	}
	/*end print wall position*/
#endif
	switch (positionZhonx.orientation)
	{
		case NORTH :
			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.right;
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.left;

				if (positionZhonx.cordinate.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.right;
				if (positionZhonx.cordinate.x > 0)
					maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.left;
			}
			if (positionZhonx.cordinate.y > 0)
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south = new_walls.front;

			maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north = new_walls.front;
			break;

		case EAST :

				if(positionZhonx.midOfCell == false)
				{
					maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south = new_walls.right;
					maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north = new_walls.left;

					if (positionZhonx.cordinate.y < (MAZE_SIZE - 1))
						maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north = new_walls.right;
					if (positionZhonx.cordinate.y > 0)
						maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south = new_walls.left;

				}
			if (positionZhonx.cordinate.x < (MAZE_SIZE - 1) )
				maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.front;
			maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.front;
			break;

		case SOUTH :

			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.right;
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.left;

				if (positionZhonx.cordinate.x > 0)
					maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.right;
				if (positionZhonx.cordinate.x < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.left;
			}
			if (positionZhonx.cordinate.y > 0)
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north = new_walls.front;
			maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south =
					new_walls.front;
			break;

		case WEST :
			if(positionZhonx.midOfCell == false)
			{
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north = new_walls.right;
				maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south = new_walls.left;


				if (positionZhonx.cordinate.y > 0)
					maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south = new_walls.right;
				if (positionZhonx.cordinate.y < (MAZE_SIZE - 1))
					maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north = new_walls.left;
			}
			if (positionZhonx.cordinate.x > 0)
				maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east = new_walls.front;
			maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west = new_walls.front;
			break;
	}
}
walls getCellState ()
{
	walls cell_condition;

	if (getWallPresence(FRONT_WALL) == false)
	{
		cell_condition.front = NO_WALL;
	}
	else
	{
		cell_condition.front = WALL_PRESENCE;
	}
	if (getWallPresence(LEFT_WALL) == false)
	{
		cell_condition.left = NO_WALL;
	}
	else
	{
		cell_condition.left = WALL_PRESENCE;
	}
	if (getWallPresence(RIGHT_WALL) == false)
	{
		cell_condition.right = NO_WALL;
	}
	else
	{
		cell_condition.right = WALL_PRESENCE;
	}
	return cell_condition;
}
