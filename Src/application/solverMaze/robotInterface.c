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
#include "middleware/controls/mazeControl/basicMoves.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"

#include "application/solverMaze/robotInterface.h"

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
#ifdef DEBUG_ROBOT_INTERFACE
		    bluetoothWaitReady();
			bluetoothPrintf("FORWARD");
#endif
			break;
		case RIGHT :
			while(hasMoveEnded() != TRUE);
			move (-90, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
			bluetoothWaitReady();
			bluetoothPrintf("RIGHT");
#endif
			while(hasMoveEnded() != TRUE);
			break;
		case UTURN :
			while(hasMoveEnded() != TRUE);
			move (180, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothWaitReady();
			bluetoothPrintf("UTURN");
#endif
			while(hasMoveEnded() != TRUE);
			break;
		case LEFT :
			while(hasMoveEnded() != TRUE);
			move (90, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
			bluetoothWaitReady();
			bluetoothPrintf("LEFT");
#endif
			while(hasMoveEnded() != TRUE);
			break;
	}
}

void move_zhonx(int direction_to_go, positionRobot *positionZhonx, int numberOfCell, char end_mid_of_case,
                    char chain)
{
    int turn = (4 + direction_to_go - positionZhonx->orientation) % 4;
    positionZhonx->orientation = direction_to_go;
    #ifdef DEBUG_ROBOT_INTERFACE
        if (positionZhonx->midOfCell == true)
        {
            bluetoothWaitReady();
			bluetoothPrintf("mid of cell ");
        }
        else
        {
            bluetoothWaitReady();
			bluetoothPrintf("half of cell ");
        }
    #endif
    if (positionZhonx->midOfCell == end_mid_of_case)
    {
        /*
         * numberOfCell-=CELL_LENGTH/2;
         * numberOfCell+=CELL_LENGTH/2;
         */
    }
    else if (positionZhonx->midOfCell == true)
    {
        #ifdef DEBUG_ROBOT_INTERFACE
            bluetoothWaitReady();
			bluetoothPrintf("start cell ");
        #endif
        moveStartCell(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
        numberOfCell--;
    }
    else // so endMidOfCase=true and positionZhonx->midOfCase=false
    {
        //moveHalfCell_IN(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION); //TODO : change that to end cell
    }
    switch (turn)
    {
        case FORWARD:
            #ifdef DEBUG_ROBOT_INTERFACE
                bluetoothWaitReady();
                bluetoothPrintf("FORWARD ");
            #endif
            break;
        case RIGHT:
            #ifdef DEBUG_ROBOT_INTERFACE
                bluetoothWaitReady();
                bluetoothPrintf("RIGHT ");
            #endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE);				//todo rotate in place
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
                bluetoothWaitReady();
			bluetoothPrintf("UTURN ");
            #endif
            if (positionZhonx->midOfCell == false)
            {
                numberOfCell--;
                moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION,
                          END_SPEED_TRANSLATION);
            }
            else
            {
                moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION,
                                          END_SPEED_TRANSLATION); //TODO : rotate 180° in place
            }
            break;
        case LEFT:
            #ifdef DEBUG_ROBOT_INTERFACE
                bluetoothWaitReady();
			bluetoothPrintf("LEFT ");
            #endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE);
                move(-90, 0, MAX_SPEED_ROTATION, 0); // TODO : rotate in place
            }
            else
            {
                moveRotateCCW90(MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
                numberOfCell--;
            }

            break;
    }
    if (numberOfCell > 0)
    {
        moveCell(numberOfCell, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
    }
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
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
                        new_walls.left;

                if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
                            new_walls.right;
                if (positionZhonx.coordinate_robot.x > 0)
                    maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                            new_walls.left;
            }
            if (positionZhonx.coordinate_robot.y > 0)
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south =
                        new_walls.front;

            maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north =
                    new_walls.front;
            break;

        case EAST:

            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north =
                        new_walls.left;

                if (positionZhonx.coordinate_robot.y < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north =
                            new_walls.right;
                if (positionZhonx.coordinate_robot.y > 0)
                    maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south =
                            new_walls.left;

            }
            if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1))
                maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                    new_walls.front;
            break;

        case SOUTH:

            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                        new_walls.left;

                if (positionZhonx.coordinate_robot.x > 0)
                    maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                            new_walls.right;
                if (positionZhonx.coordinate_robot.x < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.coordinate_robot.x + 1)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
                            new_walls.left;
            }
            if (positionZhonx.coordinate_robot.y > 0)
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south =
                    new_walls.front;
            break;

        case WEST:
            if (positionZhonx.midOfCell == false)
            {
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_north =
                        new_walls.right;
                maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_south =
                        new_walls.left;

                if (positionZhonx.coordinate_robot.y > 0)
                    maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y - 1)].wall_south =
                            new_walls.right;
                if (positionZhonx.coordinate_robot.y < (MAZE_SIZE - 1))
                    maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y + 1)].wall_north =
                            new_walls.left;
            }
            if (positionZhonx.coordinate_robot.x > 0)
                maze->cell[(int) (positionZhonx.coordinate_robot.x - 1)][(int) (positionZhonx.coordinate_robot.y)].wall_east =
                        new_walls.front;
            maze->cell[(int) (positionZhonx.coordinate_robot.x)][(int) (positionZhonx.coordinate_robot.y)].wall_west =
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
   while(getWallPresence(FRONT_WALL) == false);
   HAL_Delay(20);
   while(getWallPresence(FRONT_WALL) == true);
   HAL_Delay(500);
}

int test_move_zhonx ()
{
    positionRobot zhonx_position;
    zhonx_position.coordinate_robot.x = 8;
    zhonx_position.coordinate_robot.y = 8;
    zhonx_position.midOfCell = TRUE;
    zhonx_position.orientation = NORTH;
    coordinate way[] = {{8,7},{8,8},{8,7},{8,6},{7,6},{7,7},{7,8},{7,7},{7,6},{8,6},{8,7},{8,8},{END_OF_LIST, END_OF_LIST}}; //,{9,6},{9,7},{9,8},{9,7},{10,7},{10,6},{10,7},{END_OF_LIST,END_OF_LIST}};

    motorsInit();
    HAL_Delay(2000);
    telemetersStart();
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);
    moveRealZhonxArc(&maze, &zhonx_position, way);
    HAL_Delay(500);
    motorsDriverSleep(ON);
    telemetersStop();
    return MAZE_SOLVER_E_SUCCESS;
}

int save_maze (labyrinthe *maze)
{

    return MAZE_SOLVER_E_SUCCESS;
}
