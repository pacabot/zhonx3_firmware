/**************************************************************************/
/*!
 @file    commands.h
 @author  Netanel (PACABOT)
 @date    02/05/2015
 @version 0.1
 */
/**************************************************************************/

#ifndef __COMMANDS_H__
#define __COMMANDS_H__

/* Module Identifier */
#include "config/module_id.h"

/****************
 * Error codes
 ****************/
/* Success */
#define COMMAND_E_SUCCESS        0
/* Unspecified error */
#define COMMAND_E_ERROR          MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 1)
/* Command does not exist */
#define COMMAND_E_UNKNOWN_CMD    MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 2)
/* No callback registered for the given command */
#define COMMAND_E_NOT_REGISTERED MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 3)
/* Output function not registered */
#define COMMAND_E_NO_OUTPUT      MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 4)

/* Command handler */
typedef struct
{
    /* Command */
    const char *command;
    /* Callback function to execute the command */
    int (*pCmdCallback)(const char *args);
    /* Command description */
    const char *description;
} CMD_HANDLER;

/* Exported variables */
extern const CMD_HANDLER cmd_handlers[];

/**
 * @brief Week output function
 *
 * @retval none
 */
void cmd_output(const char *format, ...);

/**
 * @brief Displays prompt message
 *
 * @retval none
 */
void cmd_displayPrompt(void);

/*********************
 * Command functions
 *********************/
/**
 * @brief Displays the available commands
 *
 * @param   args    Pointer to command-line arguments
 *
 * @retval
 */
int cmd_help(const char *args);
int cmd_maze(const char *args);
int cmd_halt(const char *args);

#endif // __COMMANDS_H__
