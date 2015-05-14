/**************************************************************************/
/*!
    @file    cmdline_parser.c
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

#include "usart.h"
#include "stm32f4xx_hal_uart.h"

/* Declarations for this module */
#include "middleware/cmdline/cmdline_parser.h"
#include "middleware/cmdline/commands/commads.h"

#define MAX_COMMAND_LEN 20 + 1

/* Context of this module */
CMDLINE_CONTEXT cmdline_ctxt;
// Buffer used for Command line parser
extern char serial_buffer[100];

/* Private functions */
static CMD_HANDLER *cmdline_check_cmd(const char *cmd);


int cmdline_init(CMDLINE_CONTEXT *context)
{
    // Initialize Command line context
    cmdline_ctxt.cmdline = serial_buffer;
    cmdline_ctxt.cmd_received = FALSE;
    cmdline_ctxt.cmd_len = 0;

    if (context != NULL)
    {
        cmdline_ctxt.out = context->out;
    }
    else
    {
        cmdline_ctxt.out = cmd_output;
    }
    cmd_displayPrompt();

    // Enable interrupts on UART3
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);

    return CMDLINE_PARSER_E_SUCCESS;
}

int cmdline_parse(void)
{
    const char  *params = NULL;
    CMD_HANDLER *hcmd = NULL;
    char        command[MAX_COMMAND_LEN];
    char        *cmd_end;
    int         rv;

    if (cmdline_ctxt.cmd_received == FALSE)
    {
        return CMDLINE_PARSER_E_SUCCESS;
    }

    cmdline_ctxt.cmd_received = FALSE;

    // Search SPACE character
    cmd_end = strchr(cmdline_ctxt.cmdline, ' ');
    if (cmd_end == NULL)
    {
        // SPACE character not found. Search Carriage Return
        cmd_end = strchr(cmdline_ctxt.cmdline, '\r');
        if (cmd_end == NULL)
        {
            cmdline_ctxt.out("Bad format\n");
            rv = CMDLINE_PARSER_E_CMD_NOT_FOUND;
            goto out;
        }
    }
    else
    {
        // Skip space character for parameters
        params = cmd_end + 1;
    }

    memcpy(command, cmdline_ctxt.cmdline, cmd_end - cmdline_ctxt.cmdline);
    command[cmd_end - cmdline_ctxt.cmdline] = '\0';

    /****************
     * Parse command
     ****************/
    /* Check if command exists */
    hcmd = cmdline_check_cmd(command);
    if ((hcmd == NULL) || (hcmd->pCmdCallback == NULL))
    {
        cmdline_ctxt.out("Command '%s' does not exist\n", command);
        /* Command does not exist */
        rv = CMDLINE_PARSER_E_UNKNOWN_CMD;
        goto out;
    }

    /******************
     * Execute command
     ******************/
    rv = hcmd->pCmdCallback(params);

out:
    // Display back prompt message
    cmd_displayPrompt();
    return rv;
}

void cmdline_setCmdReceived(int status, int cmd_length)
{
    if ((status != TRUE) && (status != FALSE))
    {
        return;
    }
    cmdline_ctxt.cmd_received = status;
}


/**
 * @brief   Checks if command exists
 *
 * @param   cmd The command to search
 *
 * @retval  A pointer to a CMD_HANDLER on success, NULL otherwise
 */
static CMD_HANDLER *cmdline_check_cmd(const char *cmd)
{
    CMD_HANDLER *hcmd = (CMD_HANDLER *)cmd_handlers;

    /* Walk through commands array */
    while(hcmd->command != NULL)
    {
        if (strcmp(hcmd->command, cmd) == 0)
        {
            /* Command found, return a pointer to the current command handler */
            return hcmd;
        }
        hcmd++;
    }
    return NULL;
}
