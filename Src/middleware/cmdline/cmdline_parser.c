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
#include "stm32f4xx_hal_def.h"

#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"

/* Declarations for this module */
#include "middleware/cmdline/cmdline_parser.h"
#include "middleware/cmdline/commands/commads.h"

#define MAX_COMMAND_LEN 100 + 1

/* Context of this module */
CMDLINE_CONTEXT cmdline_ctxt = {
NULL,   // output callback
FALSE,  // cmd_received flag
NULL,   // Pointer to command line
0,      // command line length
FALSE   // is_initialized flag
        };
// Buffer used for Command line parser
extern char serial_buffer[100];

/* Private functions */
static CMD_HANDLER *cmdline_check_cmd(const char *cmd);

int cmdline_init(CMDLINE_CONTEXT *context)
{
#ifdef CONFIG_USE_CMDLINE
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

    cmdline_ctxt.is_initialized = TRUE;
#else
    UNUSED(context);
#endif

    return CMDLINE_PARSER_E_SUCCESS;
}

int cmdline_parse(void)
{
    const char *params = NULL;
    CMD_HANDLER *hcmd = NULL;
    char command[MAX_COMMAND_LEN];
    char *cmd_end;
    int cmd_len = 0;
    int rv;

    if (cmdline_ctxt.cmd_received == FALSE)
    {
        return CMDLINE_PARSER_E_SUCCESS;
    }

    // Reset fields
    cmdline_ctxt.cmd_received = FALSE;

    // Checks whether this is a Bluetooth event
    if (isBluetoothEvent(cmdline_ctxt.cmdline))
    {
//        static int line = 0;
//
//        ssd1306ClearScreen(MAIN_AREA);
//        ssd1306Printf(0, line * 6, &Font_3x6, "%s", cmdline_ctxt.cmdline);
//        ssd1306Refresh(MAIN_AREA);
//        line++;
//        line %= 10;
        return CMDLINE_PARSER_E_SUCCESS;
    }

    // Search for SPACE character
    cmd_end = strchr(cmdline_ctxt.cmdline, ' ');
    if (cmd_end == NULL)
    {
        // SPACE character not found. Search for Carriage Return
        cmd_end = strchr(cmdline_ctxt.cmdline, CMDLINE_CR);
        if (cmd_end == NULL)
        {
            cmdline_ctxt.out("\rBad format");
            rv = CMDLINE_PARSER_E_CMD_NOT_FOUND;
            goto out;
        }
    }
    else
    {
        // Skip space character for parameters
        params = cmd_end + 1;
    }

    // Compute actual command length
    cmd_len = cmd_end - cmdline_ctxt.cmdline;

    // Do not return error if only Carriage Return has been received
    if ((cmd_len == 0) || cmdline_ctxt.cmdline[0] == ' ')
    {
        rv = CMDLINE_PARSER_E_SUCCESS;
        goto out;
    }

    memcpy(command, cmdline_ctxt.cmdline, cmd_len);
    command[cmd_len] = '\0';

    /****************
     * Parse command
     ****************/
    /* Check if command exists */
    hcmd = cmdline_check_cmd(command);
    if ((hcmd == NULL) || (hcmd->pCmdCallback == NULL))
    {
        cmdline_ctxt.out("\rCommand '%s' does not exist", command);
        /* Command does not exist */
        rv = CMDLINE_PARSER_E_UNKNOWN_CMD;
        goto out;
    }

    /******************
     * Execute command
     ******************/
    rv = hcmd->pCmdCallback(params);

    out:
    // Reset Command line buffer
    memset(cmdline_ctxt.cmdline, 0x00, cmdline_ctxt.cmd_len);
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
    CMD_HANDLER *hcmd = (CMD_HANDLER *) cmd_handlers;

    /* Walk through commands array */
    while (hcmd->command != NULL)
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
