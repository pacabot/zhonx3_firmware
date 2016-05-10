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

STORED_MAZES *stored_mazes = (STORED_MAZES *)STORED_MAZES_ADDR;

void goOrientation(char *orientationZhonx, char directionToGo)
{
	int turn = (4 + directionToGo - *orientationZhonx) % 4;
	*orientationZhonx = directionToGo;
	switch (turn)
	{
		case FORWARD :
#ifdef DEBUG_ROBOT_INTERFACE
		    bluetoothWaitReady();
			bluetoothPrintf("FORWARD\n");
#endif
			break;
		case RIGHT :
			while(hasMoveEnded() != TRUE);
			move (-90, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
			bluetoothWaitReady();
			bluetoothPrintf("RIGHT\n");
#endif
			while(hasMoveEnded() != TRUE);
			break;
		case UTURN :
			while(hasMoveEnded() != TRUE);
			move (180, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
            bluetoothWaitReady();
			bluetoothPrintf("UTURN\n");
#endif
			while(hasMoveEnded() != TRUE);
			break;
		case LEFT :
			while(hasMoveEnded() != TRUE);
			move (90, 0, MAX_SPEED_ROTATION, 0);
#ifdef DEBUG_ROBOT_INTERFACE
			bluetoothWaitReady();
			bluetoothPrintf("LEFT\n");
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
                bluetoothPrintf("FORWARD\n");
            #endif
            break;
        case RIGHT:
            #ifdef DEBUG_ROBOT_INTERFACE
                bluetoothWaitReady();
                bluetoothPrintf("RIGHT\n");
            #endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE);				//todo rotate in place
                move(90, 0, MAX_SPEED_ROTATION, 0);
            }
            else
            {
                moveRotateCW90(MAX_SPEED_ROTATION, END_SPEED_TRANSLATION);
                numberOfCell--;
            }
            break;
        case UTURN:
            #ifdef DEBUG_ROBOT_INTERFACE
                bluetoothWaitReady();
			bluetoothPrintf("UTURN\n");
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
			bluetoothPrintf("LEFT\n");
            #endif
            if (positionZhonx->midOfCell == true)
            {
                while (hasMoveEnded() != TRUE);
                move(-90, 0, MAX_SPEED_ROTATION, 0); // TODO : rotate in place
            }
            else
            {
                moveRotateCCW90(MAX_SPEED_ROTATION, END_SPEED_TRANSLATION);
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
    positionZhonx->orientation = (positionZhonx->orientation + 2) % 4;

	moveUTurn(MAX_SPEED_ROTATION, MAX_SPEED_TRANSLATION, END_SPEED_TRANSLATION);
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


// Save a maze into the flash memory
int saveMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate)
{
    MAZE_CONTAINER maze_container;

    memcpy(&(maze_container.maze), maze, sizeof(labyrinthe));
    memcpy(&(maze_container.start_position), start_position, sizeof(positionRobot));
    memcpy(&(maze_container.end_coordinate), end_coordinate, sizeof(coordinate));
    int rv = 0;
    int selected_maze = 0;
    int cnt_mazes = stored_mazes->count_stored_mazes;

    // Check whether flash data have been initialized
    if ((cnt_mazes > 0) &&
        (cnt_mazes < MAX_STORABLE_MAZES))
    {
        // TODO: Display a menu to select a maze slot in flash memory

    }
    else
    {
        ssd1306ClearScreen(MAIN_AREA);

        // Store maze in the first slot

        // Initialize maze counter
#if 0
        ssd1306PrintfAtLine(0, 1, &Font_5x8, "Saving maze counter...");
        ssd1306Refresh();

        cnt_mazes = 1;
        rv = flash_write(zhonxSettings.h_flash, (unsigned char *)&(stored_mazes->count_stored_mazes),
                                                (unsigned char *)&cnt_mazes, sizeof(int));
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            ssd1306PrintfAtLine(0, 2, &Font_5x8, "Failed to save maze counter!");
            ssd1306Refresh();
            return rv;
        }
        ssd1306PrintfAtLine(0, 2, &Font_5x8, "Maze counter save successfully");
        ssd1306Refresh();
#endif

        // Store maze
        ssd1306PrintfAtLine(0, 3, &Font_5x8, "Saving maze...");
        ssd1306Refresh();

        rv = flash_write(zhonxSettings.h_flash, (unsigned char *)&(stored_mazes->mazes[selected_maze].maze),
                                                (unsigned char *)&maze_container, sizeof(MAZE_CONTAINER));
        if (rv != FLASH_DRIVER_E_SUCCESS)
        {
            ssd1306PrintfAtLine(0, 4, &Font_5x8, "Failed to save maze!");
            ssd1306Refresh();
            return rv;
        }
        ssd1306PrintfAtLine(0, 4, &Font_5x8, "Maze saved successfully");
        ssd1306Refresh();
    }

    return 0;
}

int loadMaze(labyrinthe *maze, positionRobot *start_position, coordinate  *end_coordinate)
{
    MAZE_CONTAINER maze_container;
    ssd1306ClearScreen(MAIN_AREA);

    // Check if there is at least one stored maze in flash memory
    if ((stored_mazes->count_stored_mazes < 0) || (stored_mazes->count_stored_mazes > MAX_STORABLE_MAZES))
    {
        ssd1306PrintfAtLine(0, 1, &Font_5x8, "Invalid stored mazes counter");
        ssd1306Refresh();
        return MAZE_SOLVER_E_ERROR;
    }

    // Get the first maze slot
    memcpy(&maze_container, &(stored_mazes->mazes[0]), sizeof(MAZE_CONTAINER));

    ssd1306PrintfAtLine(0, 1, &Font_5x8, "Maze restored successfully");
    ssd1306Refresh();

    memcpy(maze, &(maze_container.maze), sizeof(labyrinthe));
    memcpy(start_position, &(maze_container.start_position), sizeof(positionRobot));
    memcpy(end_coordinate, &(maze_container.end_coordinate), sizeof(coordinate));

    return MAZE_SOLVER_E_SUCCESS;
}

int test_move_zhonx ()
{
    labyrinthe  maze;
    positionRobot zhonx_position;
    zhonx_position.coordinate_robot.x = 8;
    zhonx_position.coordinate_robot.y = 8;
    zhonx_position.midOfCell = TRUE;
    zhonx_position.orientation = NORTH;
    coordinate way[]={{8,7},{8,6},{8,5},{8,4},{8,3},{8,2},{8,1},{8,0},{9,0},{10,0},{11,0},{10,0},{9,0},{8,0},{8,1},{8,2},{8,3},{8,4},{8,5},{8,6},{8,7},{8,8},{END_OF_LIST,END_OF_LIST}};
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
