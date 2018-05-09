/*
 * resolution_maze.h
 *
 *  Created on: 27 sept. 2014
 *      Author: Colin
 */

#ifndef RESOLUTION_MAZE_H_
#define RESOLUTION_MAZE_H_

#include <config/config.h>
#include <middleware/settings/settings.h>
#include <stddef.h>
#define NAND(a,b) (!(a) && !(b))


/* Error codes */
#define MAZE_SOLVER_E_SUCCESS  0
#define MAZE_SOLVER_E_ERROR    MAKE_ERROR(MAZE_SOLVER_MODULE_ID, 1)

#define BETWEEN(X,A,B) (((X) > (A)) && ((X) < (B)))


//definition for numberation function
/* --the value of CANT_GO can be understand like infinity weight-- */
#define INFINITY_WEIGHT 0x7FFFFFFF

/**
 * @enum direction
 * @brief direction possible
 *
 * list all orientation that the robot can have
 */
enum direction
{
  north      = 0, /*!< front to the north               */
  north_east = 1, /*!< front to the diagonal north east */
  east       = 2, /*!< front to the east                */
  south_east = 3, /*!< front to the diagonal south east */
  south      = 4, /*!< front to the south               */
  south_west = 5, /*!< front to the diagonal south east */
  west       = 6, /*!< front to the west                */
  north_west = 7, /*!< front to the diagonal north east */
};

/**
 * @enum orientation
 * @biren orientation that the robot can go
 *
 * list all the orientation that the robot can go when it do a move
 */

enum orientation
{
  forward  = 0, /*!< straight                   */
  right45  = 1, /*!< start diag right move      */
  right    = 2, /*!< right turn                 */
  right135 = 3, /*!< start diag right back move */
  uturn    = 4, /*!< uturn                      */
  left135  = 5, /*!< start diag left back move  */
  left     = 6, /*!< left turn                  */
  left45   = 7  /*!< start diag left move       */
};

//wall state define
#define NO_KNOWN 0
#define WALL_PRESENCE 1
#define NO_WALL 2

#define NO_END				0
#define END_FIND			1
#define POSSIBLE_END_FIND	2

#define PRINT_MAZE
#define PRINT_MAZE_DURING_RUN
#define PRINT_BLUETOOTH_MAZE
#define PRINT_BLUETOOTH_MAZE_DURING_RUN
#define PRINT_BLUETOOTH_BASIC_DEGUG
//#define PRINT_BLUETOOTH_ADVANCED_DEBUG
#define PRINT_WALLS_DETECTED

#define END_OF_LIST 255
#define DISPLAY_OFFSET	    12	//offset for maze print on ssd1306
#define MAX_STORABLE_MAZES  5

#include <stdlib.h>
#include "middleware/wall_sensors/wall_sensors.h"

//Structures typedef
typedef struct
{
  char wall_north;
  char wall_south;
  char wall_east;
  char wall_west;
  int length;
}one_cell;

typedef struct
{
  one_cell cell[MAZE_SIZE][MAZE_SIZE];
}labyrinthe;

typedef struct
{
    enum direction robot_direction;
    char midOfCell;
    coordinate coordinate_robot;
}positionRobot;

typedef struct
{
    // XXX: This field has been commented-out because of alignment issues in Flash memory!
    //char            maze_name[50];
    labyrinthe      maze;
    positionRobot   start_position;
    coordinate      end_coordinate;
}MAZE_CONTAINER;

typedef struct
{
    int             count_stored_mazes;
    MAZE_CONTAINER  mazes[MAX_STORABLE_MAZES];
}STORED_MAZES;

// fonctions
int maze_solver_new_maze(void);
int restartExplo();
int exploration(labyrinthe *maze, positionRobot* positionZhonx,const positionRobot *start_coordinates,
        coordinate *end_coordinate);
int startRun1(void);
int startRun2(void);
int maze_solver_run(const int runType);
int findTheShortestPath(labyrinthe *maze, positionRobot* positionZhonx,
                        const positionRobot *start_coordinates, coordinate *end_coordinate);
int goToPosition(labyrinthe *maze, positionRobot* positionZhonx,  coordinate end_coordinate);
int moveVirtualZhonx(labyrinthe maze, positionRobot positionZhonxVirtuel,
		coordinate way[], coordinate end_coordinate);
void computeCellWeight(labyrinthe *maze, coordinate end_coordinate, char wallNoKnow, char contournKnownCell);
void mazeInit (labyrinthe *maze);
void* calloc_s (size_t nombre, size_t taille);
void printMaze(labyrinthe maze, coordinate robot_coordinate);
void printLength(const labyrinthe maze,const int x_robot, const int y_robot);
void clearMazelength(labyrinthe* maze);
char miniwayFind(labyrinthe *maze, coordinate start_coordinate, coordinate end_coordinate);
int moveRealZhonxArc(labyrinthe *maze, positionRobot *positionZhonx,
                     coordinate way[], int max_speed_rotation, int max_speed_translation, int min_speed_translation);
void waitStart(void);
char diffway(coordinate way1[], coordinate way2[]);
coordinate findEndCoordinate (coordinate coordinate_tab[]);
int findArrival (labyrinthe maze, coordinate *end_coordinate);
int print_no_solution();
int printStoredMaze ();

#endif /* RESOLUTION_MAZE_H_ */
