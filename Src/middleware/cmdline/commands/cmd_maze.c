/**************************************************************************/
/*!
 @file    cmd_maze.c
 @author  Netanel (PACABOT)
 @date    14/05/2015
 @version 0.1
 */
/**************************************************************************/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "middleware/cmdline/commands/commads.h"
#include "middleware/cmdline/cmdline_parser.h"

#include "application/solverMaze/solverMaze.h"

int cmd_maze(const char *args)
{
    UNUSED(args);

    maze_solver();
    return CMDLINE_PARSER_E_SUCCESS;
}
