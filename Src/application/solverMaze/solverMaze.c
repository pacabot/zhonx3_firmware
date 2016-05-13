/*
 * @file : solverMaze.c
 * @author : Colin
 * @version : V 2.1.1
 * @date : 4 juin 2015
 * @brief : this file contain all maze solver application.
 *            this file need robotInterface.c for make hard interface and need
 *            runc.c for explain strategy for run.
 *
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "config/basetypes.h"

/* Middleware declarations */
#include "middleware/controls/lineFollowerControl/lineFollowControl.h"
#include "middleware/controls/mainControl/mainControl.h"
#include "middleware/controls/mainControl/positionControl.h"
#include "middleware/controls/mainControl/speedControl.h"
#include "middleware/controls/mainControl/transfertFunction.h"
#include "middleware/settings/settings.h"
#include "middleware/wall_sensors/wall_sensors.h"
#include "middleware/controls/pidController/pidController.h"
#include "middleware/display/pictures.h"

/* peripherale inlcudes*/
#include "peripherals/times_base/times_base.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/motors/motors.h"
#include "peripherals/lineSensors/lineSensors.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/tone/tone.h"

/* application include */
#include "application/solverMaze/solverMaze.h"
#include "application/solverMaze/robotInterface.h"
#include "application/solverMaze/run.h"

/*
 *  ********************** int maze (void) **********************
 *  this function is the main function of the maze solver
 *  return value is the succes of the maze exploration and maze run
 */
int maze_solver_new_maze(void)
{
    coordinate end_coordinate; // it's the coordinates which Zhonx have at the start
    positionRobot zhonx_position, start_position;
    labyrinthe maze;
    int rv = MAZE_SOLVER_E_SUCCESS;
    mazeInit(&maze);
    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);
#ifdef SIMULATOR
    pt_zhonx_position = &zhonx_position;
#endif // simulator

    /*init zhonx start position for nime micromouse competition*/

    zhonx_position.coordinate_robot.x = 8;
    zhonx_position.coordinate_robot.y = 8; // the robot start in the center of the maze
    zhonx_position.orientation = zhonxSettings.start_orientation % 4;
    /*end of initialization for nime micromouse competition*/
    zhonx_position.midOfCell = true;
    memcpy(&start_position, &zhonx_position, sizeof(positionRobot));
    start_position.orientation += 2;
    start_position.orientation = start_position.orientation % 4;
    start_position.midOfCell = false;
    //newCell(ask_cell_state(), &maze, start_position);
    newCell((walls){WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE,WALL_PRESENCE}, &maze, start_position);
    memcpy(&start_position, &zhonx_position, sizeof(positionRobot));

    printLength(maze,8,8);
#ifdef PRINT_MAZE_DURING_RUN
    printMaze(maze, zhonx_position.coordinate_robot);
#endif

    telemetersStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(30, 0, &Font_5x8, "WAIT START...");
    ssd1306Refresh();
    waitStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "SCAN...");
    ssd1306Refresh();

    rv = exploration(&maze, &zhonx_position, &start_position, &end_coordinate); //make exploration for go from the robot position and the end of the maze
    if (rv != MAZE_SOLVER_E_SUCCESS)
    {
        bluetoothWaitReady();
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
        bluetoothPrintf("no solution");
#endif
        tone(D4, 200);
        HAL_Delay(50);
        tone(C4H, 100);
        HAL_Delay(40);
        toneItMode(C4, 150);
        ssd1306WaitReady();
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawBmp(warning_Img, 1, 15, 48, 43);
        ssd1306PrintfAtLine(55, 1, &Font_5x8, "NO SOLUTION");
        ssd1306Refresh();
    }
    else
    {
        findTheShortestPath(&maze, &zhonx_position, &start_position, &end_coordinate); //find the shortest path
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(0, 0, &Font_5x8, "TARGET REACHED !!!");
        ssd1306Refresh();
        tone(G5, 100);
        HAL_Delay(100);
        tone(A5, 100);
        HAL_Delay(100);
        tone(B5, 50);
        HAL_Delay(50);
        toneItMode(B5, 400);
    }
    motorsDriverSleep(ON); //because flash write cause interrupts damages
    telemetersStop();//because flash write cause interrupts damages
    rv=saveMaze(&maze, &start_position, &end_coordinate);
    if (rv != FLASH_DRIVER_E_SUCCESS)
    {
    }
    telemetersStart();//because flash write cause interrupts damages

    ssd1306PrintfAtLine(0,0,&Font_5x8,"go to start position");
    ssd1306Refresh();
    goToPosition(&maze, &zhonx_position, start_position.coordinate_robot);	//goto start position
    // Save maze into flash memory

    doUTurn(&zhonx_position); //initial position
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothPrintf("uturn do\ngo back");
#endif
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(40, 0, &Font_5x8, "END SEARCH");
    ssd1306Refresh();
    telemetersStop();
    motorsDriverSleep(ON);
    HAL_Delay(1000);
#ifdef PRINT_MAZE
    ssd1306ClearScreen(MAIN_AREA);
    printLength(maze,8,8);
    printMaze(maze, zhonx_position.coordinate_robot);
#endif
    while (expanderJoyFiltered() != JOY_LEFT);
    return MAZE_SOLVER_E_SUCCESS;
}

int maze_solver_run ()
{
    labyrinthe maze;
    positionRobot zhonx_position, start_position;
    coordinate end_coordinate;

    mainControlSetFollowType(WALL_FOLLOW);
    positionControlSetPositionType(GYRO);
    memcpy(&zhonx_position, &start_position,sizeof(positionRobot));

    loadMaze(&maze, &start_position, &end_coordinate);

#ifdef PRINT_MAZE
    ssd1306ClearScreen(MAIN_AREA);
    printLength(maze,8,8);
    printMaze(maze, zhonx_position.coordinate_robot);
#endif

    telemetersStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(30, 0, &Font_5x8, "WAIT START...");
    ssd1306Refresh();
    waitStart();
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(30, 0, &Font_5x8, "RUN...");
    ssd1306Refresh();

    run1(&maze, &zhonx_position,
         start_position.coordinate_robot,
         end_coordinate);

    run2(&maze, &zhonx_position,
         start_position.coordinate_robot,
         end_coordinate);

    HAL_Delay(100);
    motorsDriverSleep(ON);
    return MAZE_SOLVER_E_SUCCESS;
}

int exploration(labyrinthe *maze, positionRobot* positionZhonx,
                const positionRobot *start_coordinates, coordinate *end_coordinate)
{
    int rv = MAZE_SOLVER_E_SUCCESS;
    coordinate way[MAZE_SIZE * MAZE_SIZE] = { { -1, -1 }, { END_OF_LIST,
            END_OF_LIST } };
    coordinate last_coordinate;
    poids(maze, positionZhonx->coordinate_robot, true, false);

    while (findArrival(*maze, end_coordinate) != MAZE_SOLVER_E_SUCCESS)
    {
        newCell(getCellState(), maze, *positionZhonx);
        clearMazelength(maze);
        poids(maze, *end_coordinate, true, true);
        rv = moveVirtualZhonx(*maze, *positionZhonx, way, *end_coordinate);
        if (rv != MAZE_SOLVER_E_SUCCESS)
        {
            moveStop();
#ifdef PRINT_MAZE_DURING_RUN
            printMaze(*maze, positionZhonx->coordinate_robot);
#endif
            printLength(*maze, -1, -1);
            return rv;
        }
        rv = moveRealZhonxArc(maze, positionZhonx, way);
        if (rv != MAZE_SOLVER_E_SUCCESS)
        {
            moveStop();
            ssd1306ClearScreen(MAIN_AREA);
            ssd1306DrawBmp(warning_Img, 1, 15, 48, 43);
            ssd1306PrintfAtLine(55, 1, &Font_5x8, "ERROR WAY");
            ssd1306Refresh();
            return rv;
        }
        clearMazelength(maze);
        poids(maze, positionZhonx->coordinate_robot, true, false);
        printLength(*maze, positionZhonx->coordinate_robot.x,
                    positionZhonx->coordinate_robot.y);
    }
    last_coordinate = findEndCoordinate(way);
    moveStop();

    return rv;
}

int findTheShortestPath(labyrinthe *maze, positionRobot* positionZhonx,
                        const positionRobot *start_coordinates, coordinate *end_coordinate)
{
    int rv = MAZE_SOLVER_E_SUCCESS;
    coordinate way[MAZE_SIZE * MAZE_SIZE] = { { -1, -1 }, { END_OF_LIST,
            END_OF_LIST } };
    coordinate last_coordinate;
    do
    {
        clearMazelength(maze);
        poids(maze, *end_coordinate, true, false);
        rv = moveVirtualZhonx(*maze, *start_coordinates, way, *end_coordinate);
        if (rv == MAZE_SOLVER_E_SUCCESS)
        {
            last_coordinate = findEndCoordinate(way);
            if (last_coordinate.x == end_coordinate->x
                    && last_coordinate.y == end_coordinate->y)
                break;
            goToPosition(maze, positionZhonx, last_coordinate);
        }
    } while ((last_coordinate.x != end_coordinate->x)
            || (last_coordinate.y != end_coordinate->y));
    return rv;
}

int goToPosition(labyrinthe *maze, positionRobot* positionZhonx,
                 coordinate end_coordinate)
{
    coordinate way[MAZE_SIZE * MAZE_SIZE];
    int rv;
    newCell(getCellState(), maze, *positionZhonx);
    while (positionZhonx->coordinate_robot.x != end_coordinate.x
            || positionZhonx->coordinate_robot.y != end_coordinate.y)
    {
        clearMazelength(maze);	// clear the length for make it with new walls
        poids(maze, end_coordinate, true, false);// calculate length into the new maze
        rv = moveVirtualZhonx(*maze, *positionZhonx, way, end_coordinate);// create way for go to the end coordinate if it possible
        if (rv != MAZE_SOLVER_E_SUCCESS)
        {
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
            bluetoothWaitReady();
            bluetoothPrintf("no solution");
#endif
            while (1);// todo debug
            // no solution for go to the asked position
            return rv;
        }
        rv = moveRealZhonxArc(maze, positionZhonx, way);	// use way for go the end position or closer position if there are no-know wall
        if (rv != MAZE_SOLVER_E_SUCCESS)
        {
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
            bluetoothWaitReady();
            bluetoothPrintf("error way");
#endif
            while (1); //todo debug
            // no solution for go to the asked position
            return rv;
        }
    }
    return MAZE_SOLVER_E_SUCCESS;
}

int moveVirtualZhonx(labyrinthe maze, positionRobot positionZhonxVirtuel,
                     coordinate way[], coordinate end_coordinate)
{
    int i = 0;
    while (positionZhonxVirtuel.coordinate_robot.x != end_coordinate.x
            || positionZhonxVirtuel.coordinate_robot.y != end_coordinate.y)
    {
        if (maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x + 1)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length
                < maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length&& positionZhonxVirtuel.coordinate_robot.x+1<MAZE_SIZE
                && maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].wall_east==NO_WALL)
        {
            positionZhonxVirtuel.coordinate_robot.x = positionZhonxVirtuel.coordinate_robot.x
                    + 1;
        }
        else if (maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y
                + 1)].length
                < maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length&& positionZhonxVirtuel.coordinate_robot.y+1<MAZE_SIZE
                && maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].wall_south==NO_WALL)
        {
            positionZhonxVirtuel.coordinate_robot.y = positionZhonxVirtuel.coordinate_robot.y
                    + 1;
        }
        else if (maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x - 1)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length
                < maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length&& positionZhonxVirtuel.coordinate_robot.x>0
                && maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].wall_west==NO_WALL)
        {
            positionZhonxVirtuel.coordinate_robot.x = positionZhonxVirtuel.coordinate_robot.x
                    - 1;
        }
        else if (maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y
                - 1)].length
                < maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].length&& positionZhonxVirtuel.coordinate_robot.y>0
                && maze.cell[(int)(positionZhonxVirtuel.coordinate_robot.x)][(int)(positionZhonxVirtuel.coordinate_robot.y)].wall_north==NO_WALL)
        {
            positionZhonxVirtuel.coordinate_robot.y = positionZhonxVirtuel.coordinate_robot.y
                    - 1;
        }
        else
        {
            way[i].x = END_OF_LIST, way[i].y = END_OF_LIST;
            if (i != 0)
            {
                return MAZE_SOLVER_E_SUCCESS;
            }
            else
            {
                return MAZE_SOLVER_E_ERROR;
            }
        }
#ifdef PRINT_MAZE_DURING_RUN
        printMaze(maze, positionZhonxVirtuel.coordinate_robot);
#endif
        way[i].x = positionZhonxVirtuel.coordinate_robot.x, way[i].y =
                positionZhonxVirtuel.coordinate_robot.y;
        i++;
    }
    way[i].x = END_OF_LIST, way[i].y = END_OF_LIST;
    return MAZE_SOLVER_E_SUCCESS;
}

int moveRealZhonxArc(labyrinthe *maze, positionRobot *positionZhonx,
                     coordinate way[])
{
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothWaitReady();
    bluetoothPrintf("pat");
#endif
    walls cell_state;
    char chain;
    int nb_move;
    int i = 0;
    char additionY = 0;
    char additionX = 0;
    char orientaionToGo = NORTH;
    while (way[i].x != END_OF_LIST)
    {
        nb_move = 0;
        if (way[i].x == (positionZhonx->coordinate_robot.x + 1)
                && way[i].y == positionZhonx->coordinate_robot.y)
        {
            additionX = 1;
            additionY = 0;
            orientaionToGo = EAST;
        }
        else if (way[i].x == (positionZhonx->coordinate_robot.x - 1)
                && way[i].y == positionZhonx->coordinate_robot.y)
        {
            additionX = -1;
            additionY = 0;
            orientaionToGo = WEST;
        }
        else if (way[i].y == (positionZhonx->coordinate_robot.y - 1)
                && way[i].x == positionZhonx->coordinate_robot.x)
        {

            additionX = 0;
            additionY = -1;
            orientaionToGo = NORTH;
        }
        else if (way[i].y == (positionZhonx->coordinate_robot.y + 1)
                && way[i].x == positionZhonx->coordinate_robot.x)
        {

            additionX = 0;
            additionY = 1;
            orientaionToGo = SOUTH;
        }
        else
        {
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
            bluetoothWaitReady();
            bluetoothPrintf("Error way : position zhonx x= %d y=%d \t way x= %d y=%d \n",
                            positionZhonx->coordinate_robot.x,positionZhonx->coordinate_robot.y, way[i].x, way[i].y);
#endif
            return MAZE_SOLVER_E_ERROR;
        }

        while ((way[i].x != END_OF_LIST)
                && way[i].y == (positionZhonx->coordinate_robot.y + additionY)
                && way[i].x == positionZhonx->coordinate_robot.x + additionX)
        {
            positionZhonx->coordinate_robot.x = way[i].x;
            positionZhonx->coordinate_robot.y = way[i].y;
            i++;
            nb_move++;
        }
        if (way[i].x == END_OF_LIST)
            chain = false;
        else
            chain = true;
        move_zhonx(orientaionToGo, positionZhonx, nb_move, false, true);
        cell_state = getCellState();
        newCell(cell_state, maze, *positionZhonx);

    }
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
    bluetoothWaitReady();
    bluetoothPrintf("colin");
#endif
    return MAZE_SOLVER_E_SUCCESS;
}

void poids(labyrinthe *maze, coordinate end_coordinate, char wallNoKnow,
           char contournKnownCell)
{
    int i1 = 0, i2 = 0;
    int length = 0;
    int x = end_coordinate.x;
    int y = end_coordinate.y;
    maze->cell[x][y].length = length;
    coordinate dotes_to_verify_tab[MAZE_SIZE * MAZE_SIZE];
    dotes_to_verify_tab[0].x = x;
    dotes_to_verify_tab[0].y = y;
    dotes_to_verify_tab[1].x = END_OF_LIST;
    coordinate new_dotes_to_verify_tab[MAZE_SIZE * MAZE_SIZE];
    coordinate *dotes_to_verify = dotes_to_verify_tab;
    coordinate *new_dotes_to_verify = new_dotes_to_verify_tab;
    coordinate *pt = NULL;

    while (dotes_to_verify[0].x != END_OF_LIST)
    {
        while (dotes_to_verify[i1].x != END_OF_LIST)
        {
            x = dotes_to_verify[i1].x;
            y = dotes_to_verify[i1].y;
            int part_length = 0;
            if (contournKnownCell)
            {
                if (maze->cell[x][y].wall_north != NO_KNOWN)
                {
                    part_length += zhonxSettings.wall_know_cost;
                }
                if (maze->cell[x][y].wall_east != NO_KNOWN)
                {
                    part_length += zhonxSettings.wall_know_cost;
                }
                if (maze->cell[x][y].wall_south != NO_KNOWN)
                {
                    part_length += zhonxSettings.wall_know_cost;
                }
                if (maze->cell[x][y].wall_west != NO_KNOWN)
                {
                    part_length += zhonxSettings.wall_know_cost;
                }
            }
            maze->cell[x][y].length += part_length;
            length = maze->cell[x][y].length + zhonxSettings.cell_cost;

            if ((maze->cell[x][y].wall_north == NO_WALL
                    || (wallNoKnow == true
                            && maze->cell[x][y].wall_north == NO_KNOWN))
                    && maze->cell[x][y - 1].length > length - 1 && y > 0)
            {
                new_dotes_to_verify[i2].x = x;
                new_dotes_to_verify[i2].y = y - 1;
                i2++;
                maze->cell[x][y - 1].length = length;
            }
            if ((maze->cell[x][y].wall_east == NO_WALL
                    || (wallNoKnow == true
                            && maze->cell[x][y].wall_east == NO_KNOWN))
                    && maze->cell[x + 1][y].length > length&& x+1<MAZE_SIZE)
            {
                new_dotes_to_verify[i2].x = x + 1;
                new_dotes_to_verify[i2].y = y;
                i2++;
                maze->cell[x + 1][y].length = length;
            }
            if ((maze->cell[x][y].wall_south == NO_WALL
                    || (wallNoKnow == true
                            && maze->cell[x][y].wall_south == NO_KNOWN))
                    && maze->cell[x][y + 1].length > length&& y+1<MAZE_SIZE)
            {
                new_dotes_to_verify[i2].x = x;
                new_dotes_to_verify[i2].y = y + 1;
                i2++;
                maze->cell[x][y + 1].length = length;
            }
            if ((maze->cell[x][y].wall_west == NO_WALL
                    || (wallNoKnow == true
                            && maze->cell[x][y].wall_west == NO_KNOWN))
                    && maze->cell[x - 1][y].length > length && x > 0)
            {
                new_dotes_to_verify[i2].x = x - 1;
                new_dotes_to_verify[i2].y = y;
                i2++;
                maze->cell[x - 1][y].length = length;
            }
            i1++;
        }
        new_dotes_to_verify[i2].x = END_OF_LIST;
        pt = dotes_to_verify;
        dotes_to_verify = new_dotes_to_verify;
        new_dotes_to_verify = pt;
        i2 = 0;
        i1 = 0;
    }
}

void mazeInit(labyrinthe *maze)
{
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
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        maze->cell[i][0].wall_north = WALL_PRESENCE;
        maze->cell[i][MAZE_SIZE - 1].wall_south = WALL_PRESENCE;
        maze->cell[0][i].wall_west = WALL_PRESENCE;
        maze->cell[MAZE_SIZE - 1][i].wall_east = WALL_PRESENCE;
    }

}

void printMaze(labyrinthe maze, coordinate robot_coordinate)
{
    ssd1306ClearRect(0, 0, 64, 64);
    int size_cell_on_oled = ((63) / MAZE_SIZE);
    int x, y;
    for (y = 0; y < MAZE_SIZE; y++)
    {
        for (x = 0; x < MAZE_SIZE; x++)
        {
            if (maze.cell[x][y].wall_north == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                                y * size_cell_on_oled + HEAD_MARGIN,
                                x * size_cell_on_oled + size_cell_on_oled + 1,
                                y * size_cell_on_oled + HEAD_MARGIN);
            }
            else if (maze.cell[x][y].wall_north == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                                      y * size_cell_on_oled + HEAD_MARGIN,
                                      x * size_cell_on_oled + size_cell_on_oled + 1,
                                      y * size_cell_on_oled + HEAD_MARGIN);
            }
            if (maze.cell[x][y].wall_west == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                                y * size_cell_on_oled + HEAD_MARGIN,
                                x * size_cell_on_oled,
                                (y + 1) * size_cell_on_oled
                                + HEAD_MARGIN + 1);
            }
            else if (maze.cell[x][y].wall_west == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                                      y * size_cell_on_oled+ HEAD_MARGIN,
                                      x * size_cell_on_oled,
                                      (y + 1) * size_cell_on_oled
                                      + HEAD_MARGIN + 1);
            }
            if (maze.cell[x][y].wall_south == WALL_PRESENCE)
            {
                ssd1306DrawLine(x * size_cell_on_oled,
                                (y+ 1) * size_cell_on_oled+ HEAD_MARGIN,
                                size_cell_on_oled + x * size_cell_on_oled,
                                (y+ 1) * size_cell_on_oled + HEAD_MARGIN);
            }
            else if (maze.cell[x][y].wall_south == NO_KNOWN)
            {
                ssd1306DrawDashedLine(x * size_cell_on_oled,
                                      (y + 1) * size_cell_on_oled + HEAD_MARGIN,
                                      size_cell_on_oled + x * size_cell_on_oled,
                                      (y + 1) * size_cell_on_oled + HEAD_MARGIN);
            }
            if (maze.cell[x][y].wall_east == WALL_PRESENCE)
            {
                ssd1306DrawLine((x + 1) * size_cell_on_oled,
                                y * size_cell_on_oled + HEAD_MARGIN,
                                (x + 1) * size_cell_on_oled,
                                (y + 1) * size_cell_on_oled
                                + HEAD_MARGIN + 1);
            }
            else if (maze.cell[x][y].wall_east == NO_KNOWN)
            {
                ssd1306DrawDashedLine((x + 1) * size_cell_on_oled,
                                      y * size_cell_on_oled + HEAD_MARGIN,
                                      (x + 1) * size_cell_on_oled,
                                      (y + 1) * size_cell_on_oled
                                      + HEAD_MARGIN + 1);
            }
        }
    }
    printLength(maze, robot_coordinate.x, robot_coordinate.y);
    ssd1306DrawRect((robot_coordinate.x * size_cell_on_oled) + 1,
                    (robot_coordinate.y * size_cell_on_oled + HEAD_MARGIN) + 1, 2,
                    2);
    ssd1306Refresh();
}

void printLength(const labyrinthe maze, const int x_robot, const int y_robot)
{
#ifdef PRINT_BLUETOOTH_MAZE
    bluetoothWaitReady();
    bluetoothPrintf("zhonx : %d; %d\n", x_robot, y_robot);
    bluetoothPrintf("  ");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        bluetoothPrintf("%5d", i);
    }
    bluetoothPrintf("\n");
    for (int i = 0; i < MAZE_SIZE; i++)
    {
        bluetoothPrintf("    ");
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            if (maze.cell[j][i].wall_north == NO_KNOWN)
            {
                bluetoothPrintf("- - +");
            }
            else if (maze.cell[j][i].wall_north == WALL_PRESENCE)
            {
                bluetoothPrintf("----+");
            }
            else
            {
                bluetoothPrintf("    +");
            }
        }
        bluetoothPrintf("\n ");

        bluetoothPrintf("%2d ", i);
        for (int j = 0; j < MAZE_SIZE; j++)
        {
            if (maze.cell[j][i].length != CANT_GO)
            {
                bluetoothPrintf("%4d", maze.cell[j][i].length);
            }
            else
            {
                bluetoothPrintf("    ");
            }
            if (maze.cell[j][i].wall_east == NO_KNOWN)
            {
                bluetoothPrintf("!");
            }
            else if (maze.cell[j][i].wall_east == WALL_PRESENCE)
            {
                bluetoothPrintf("|");
            }
            else
            {
                bluetoothPrintf(" ");
            }
        }
        bluetoothPrintf("\n");
    }
    bluetoothPrintf("\n");
#endif
}

void clearMazelength(labyrinthe* maze)
{
    int x, y;
    for (y = 0; y < MAZE_SIZE; y++)
    {
        for (x = 0; x < MAZE_SIZE; x++)
        {
            maze->cell[x][y].length = CANT_GO;
        }
    }
}

char miniwayFind(labyrinthe *maze, coordinate start_coordinate,
                 coordinate end_coordinate)
{
    // TODO not find the shorter in distance way but the faster
    coordinate way1[MAZE_SIZE * MAZE_SIZE];
    coordinate way2[MAZE_SIZE * MAZE_SIZE];
    clearMazelength(maze);
    poids(maze, end_coordinate, true, false);
#ifdef PRINT_MAZE
    printMaze(*maze, (coordinate ) { -1, -1 });
#endif
    positionRobot position;
    position.midOfCell = true;
    position.coordinate_robot = start_coordinate;
    position.orientation = NORTH;
    moveVirtualZhonx(*maze, position, way1, end_coordinate);
    clearMazelength(maze);
    poids(maze, end_coordinate, false, false);
#ifdef PRINT_MAZE
    printMaze(*maze, (coordinate ) { -1, -1 });
#endif
    moveVirtualZhonx(*maze, position, way2, end_coordinate);
    ssd1306ClearScreen(MAIN_AREA);
    char waySame = diffway(way1, way2);
    switch (waySame)
    {
        case true:
            ssd1306DrawString(0, 20, "2 way = : yes", &Font_5x8);
            break;
        case false:
            ssd1306DrawString(0, 20, "2 way = : no", &Font_5x8);
            break;
    }
    ssd1306Refresh();
    HAL_Delay(3000);
    return (waySame);
}

char diffway(coordinate way1[], coordinate way2[])
{
    int i = 0;
    while ((way1[i].x != END_OF_LIST) && (way2[i].x != END_OF_LIST))
    {
        if (way1[i].x != way2[i].x || way1[i].y != way2[i].y)
        {
            return false;
        }
        i++;
    }
    if (!NAND((way1[i].x != END_OF_LIST), (way2[i].x != END_OF_LIST)))
    {
        return false;
    }
    return true;
}
coordinate findEndCoordinate(coordinate coordinate_tab[])
{
    int i = 0;
    while (coordinate_tab[i].x != END_OF_LIST)
    {
        i++;
    }
    return coordinate_tab[i - 1];
}

int findArrival(labyrinthe maze, coordinate *end_coordinate)
{
    int possible_end_find_cost = CANT_GO;
    for (int x = 0; x < (MAZE_SIZE - 1); x++)
    {
        for (int y = 0; y < (MAZE_SIZE - 1); ++y)
        {
            if (maze.cell[x][y].wall_east == NO_WALL
                    && maze.cell[x][y].wall_south == NO_WALL
                    && maze.cell[x][y + 1].wall_east == NO_WALL
                    && maze.cell[x + 1][y].wall_south == NO_WALL
                    && maze.cell[x][y].length != CANT_GO)
            {
                end_coordinate->x = x;
                end_coordinate->y = y;
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
                //                bluetoothPrintf("end find at : %i; %i\n", x, y);
#endif
                return MAZE_SOLVER_E_SUCCESS;
            }
            if (possible_end_find_cost
                    > maze.cell[x][y].length&&
                    maze.cell[x][y].wall_east != WALL_PRESENCE && maze.cell[x][y].wall_south != WALL_PRESENCE
                    && maze.cell[x][y+1].wall_east != WALL_PRESENCE && maze.cell[x+1][y].wall_south != WALL_PRESENCE
                    && maze.cell[x][y].length != CANT_GO
                    && maze.cell[x][y].length != CANT_GO && maze.cell[x+1][y].length != CANT_GO
                    && maze.cell[x][y+1].length != CANT_GO && maze.cell[x+1][y+1].length != CANT_GO)
            {
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
                //bluetoothPrintf("possible end find at : %i; %i\n", x, y);
#endif
                if ((maze.cell[x][y].wall_east != NO_WALL
                        || maze.cell[x][y].wall_south != NO_WALL)
                        && maze.cell[x][y].length != 0
                        && maze.cell[x][y].length < possible_end_find_cost)
                {
                    end_coordinate->x = x;
                    end_coordinate->y = y;
                    possible_end_find_cost =
                            maze.cell[end_coordinate->x][end_coordinate->y].length;
                }
                if ((maze.cell[x + 1][y].wall_west != NO_WALL
                        || maze.cell[x + 1][y].wall_south != NO_WALL)
                        && maze.cell[x+1][y].length != 0
                        && maze.cell[x+1][y].length < possible_end_find_cost)
                {
                    end_coordinate->x = x + 1;
                    end_coordinate->y = y;
                    possible_end_find_cost =
                            maze.cell[end_coordinate->x][end_coordinate->y].length;
                }
                if ((maze.cell[x][y + 1].wall_east != NO_WALL
                        || maze.cell[x][y + 1].wall_south != NO_WALL)
                        && maze.cell[x][y+1].length != 0
                        && maze.cell[x][y+1].length < possible_end_find_cost)
                {
                    end_coordinate->x = x;
                    end_coordinate->y = y + 1;
                    possible_end_find_cost =
                            maze.cell[end_coordinate->x][end_coordinate->y].length;
                }
                if ((maze.cell[x + 1][y + 1].wall_west != NO_WALL
                        || maze.cell[x + 1][y + 1].wall_south != NO_WALL)
                        && maze.cell[x+1][y+1].length != 0
                        && maze.cell[x+1][y+1].length < possible_end_find_cost)
                {
                    end_coordinate->x = x + 1;
                    end_coordinate->y = y + 1;
                    possible_end_find_cost =
                            maze.cell[end_coordinate->x][end_coordinate->y].length;
                }
#ifdef PRINT_BLUETOOTH_BASIC_DEGUG
                //bluetoothPrintf("possible end find at : %i; %i go to position : %d; %d\n", x, y, end_coordinate->x, end_coordinate->y);
#endif
            }

        }
    }
    return MAZE_SOLVER_E_ERROR;
}
