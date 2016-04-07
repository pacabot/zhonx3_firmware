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
#include "peripherals/bluetooth/bluetooth.h"

/* meddleware include */
#include "application/solverMaze/solverMaze.h"

/* Middleware declarations */
#include "middleware/controls/mazeControl/basicMoves.h"
#include "middleware/controls/mainControl/mainControl.h"

#include "application/solverMaze/robotInterface.h"

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
			bluetoothPrintf("FORWARD");
			break;
		case RIGHT :
			while(hasMoveEnded() != TRUE);
			move (-90, 0, MAX_SPEED_ROTATION, 0);
			bluetoothPrintf("RIGHT");
			while(hasMoveEnded() != TRUE);
			break;
		case UTURN :
			while(hasMoveEnded() != TRUE);
			move (180, 0, MAX_SPEED_ROTATION, 0);
			bluetoothPrintf("UTURN");
			while(hasMoveEnded() != TRUE);
			break;
		case LEFT :
			while(hasMoveEnded() != TRUE);
			move (90, 0, MAX_SPEED_ROTATION, 0);
			bluetoothPrintf("LEFT");
			while(hasMoveEnded() != TRUE);
			break;
	}
}

void move_zhonx_arc(int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case,
                    char chain)
{
    int turn = (4 + direction_to_go - positionZhonx->orientation) % 4;
    positionZhonx->orientation = direction_to_go;
    switch (turn)
    {
        case FORWARD:
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothPrintf("FORWARD \n");
#endif
            break;
        case RIGHT:
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothPrintf("RIGHT \n");
#endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE)
                    ;				//todo rotate in place
                move(90, 0, MAX_SPEED_ROTATION, 0);
            }
            else
            {
                moveRotateCW90(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
                numberOfCell--;

            }
            break;
        case UTURN:
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothPrintf("UTURN \n");
#endif
            if (positionZhonx->midOfCell == false)
            {
                numberOfCell--;
            }
            moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION,
                      END_SPEED_TRANSLATION);
            break;
        case LEFT:
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothPrintf("LEFT \n");
#endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE)
                    ;
                move(-90, 0, MAX_SPEED_ROTATION, 0);
            }
            else
            {
                moveRotateCCW90(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
                numberOfCell--;
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
        numberOfCell--;
    }
    else // so endMidOfCase=true and positionZhonx->midOfCase=false
    {
 //       moveHalfCell_IN(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION); //todo add finish move?
    }
    moveCell(numberOfCell, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
    positionZhonx->midOfCell = end_mid_of_case;

}

void doUTurn(positionRobot *positionZhonx)
{
    motorsDriverSleep(OFF);
    goOrientation(&positionZhonx->orientation, (positionZhonx->orientation + 2) % 4);

	moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
	motorsDriverSleep(ON);
}

int waitValidation(unsigned long timeout)
{
    timeout += HAL_GetTick();
    do
    {
        switch (expanderJoyFiltered())
        {
            case JOY_RIGHT:
                return JOY_RIGHT;
                break;
            case JOY_LEFT:
                return JOY_LEFT;
                break;
        }
    }
    while (timeout > HAL_GetTick());
    return -1;
}

void newCell(walls new_walls, labyrinthe *maze, positionRobot positionZhonx)
{
#ifdef DEBUG
	print_cell_state(new_walls);
#endif
    switch (positionZhonx.orientation)
    {
        case NORTH:
            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west =
                        new_walls.left;

                if (positionZhonx.cordinate.x < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west =
                            new_walls.right;
                if (positionZhonx.cordinate.x > 0)
                    maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east =
                            new_walls.left;
            }
            if (positionZhonx.cordinate.y > 0)
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south =
                        new_walls.front;

            maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north =
                    new_walls.front;
            break;

        case EAST:

            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north =
                        new_walls.left;

                if (positionZhonx.cordinate.y < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north =
                            new_walls.right;
                if (positionZhonx.cordinate.y > 0)
                    maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south =
                            new_walls.left;

            }
            if (positionZhonx.cordinate.x < (MAZE_SIZE - 1))
                maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east =
                    new_walls.front;
            break;

        case SOUTH:

            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_east =
                        new_walls.left;

                if (positionZhonx.cordinate.x > 0)
                    maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east =
                            new_walls.right;
                if (positionZhonx.cordinate.x < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.cordinate.x + 1)][(int) (positionZhonx.cordinate.y)].wall_west =
                            new_walls.left;
            }
            if (positionZhonx.cordinate.y > 0)
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south =
                    new_walls.front;
            break;

        case WEST:
            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_north =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_south =
                        new_walls.left;

                if (positionZhonx.cordinate.y > 0)
                    maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y - 1)].wall_south =
                            new_walls.right;
                if (positionZhonx.cordinate.y < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y + 1)].wall_north =
                            new_walls.left;
            }
            if (positionZhonx.cordinate.x > 0)
                maze->cell[(int) (positionZhonx.cordinate.x - 1)][(int) (positionZhonx.cordinate.y)].wall_east =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.cordinate.x)][(int) (positionZhonx.cordinate.y)].wall_west =
                    new_walls.front;
            break;
    }
}
walls getCellState()
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
walls ask_cell_state ()
{
	walls cell_state;
	memset(&cell_state, NO_KNOWN, sizeof(walls));
	int joystick = expanderJoyFiltered();
	while (joystick != JOY_UP)
	{
		joystick = expanderJoyFiltered();
		switch (joystick) {
			case JOY_DOWN:
				if (cell_state.front == WALL_PRESENCE)
				{
					cell_state.front = NO_WALL;
				}
				else
				{
					cell_state.front = WALL_PRESENCE;
				}
				break;
			case JOY_RIGHT:
				if (cell_state.left == WALL_PRESENCE)
				{
					cell_state.left = NO_WALL;
				}
				else
				{
					cell_state.left = WALL_PRESENCE;
				}
				break;
			case JOY_LEFT:
				if (cell_state.right == WALL_PRESENCE)
				{
					cell_state.right = NO_WALL;
				}
				else
				{
					cell_state.right = WALL_PRESENCE;
				}
				break;
			default:
				break;
		}
		print_cell_state(cell_state);
		ssd1306Refresh();
	}
	return cell_state;
}

void print_cell_state (walls cell_state)
{
	ssd1306ClearRect(64,DISPLAY_OFFSET,54,5);
	ssd1306ClearRect(64,DISPLAY_OFFSET,5,54);
	ssd1306ClearRect(113,DISPLAY_OFFSET,5,54);

	if (cell_state.front == WALL_PRESENCE)
	{
		ssd1306FillRect(64,DISPLAY_OFFSET,54,5);
	}
	if (cell_state.left == WALL_PRESENCE)
	{
		ssd1306FillRect(64,DISPLAY_OFFSET,5,54);
	}
	if (cell_state.right == WALL_PRESENCE)
	{
		ssd1306FillRect(113,DISPLAY_OFFSET,5,54);
	}
}

void waitStart()
{
   telemetersStart();
   HAL_Delay(300);
   while(getWallPresence(FRONT_WALL) == false);
   HAL_Delay(100);
   while(getWallPresence(FRONT_WALL) == true);
   telemetersStop();
}
