/**************************************************************************/
/*!
    @file    commands.c
    @author  Netanel (PACABOT)
    @date    02/05/2015
    @version 0.1
 */
/**************************************************************************/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

// Declarations for this module
#include "middleware/cmdline/cmdline_parser.h"
#include "middleware/cmdline/commands/commads.h"


/* Array of registered commands */
const CMD_HANDLER cmd_handlers[] =
{
        {"help", cmd_help, "Displays the available commands"},
        {"maze", cmd_maze, "Solve maze"},

        {NULL, NULL, NULL}  // This element is mandatory. It indicates the end of the array
};

void cmd_output(const char *format, ...)
{
    return;
}

void cmd_displayPrompt(void)
{
    cmdline_ctxt.out("\n> ");
}
