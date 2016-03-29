/**************************************************************************/
/*!
 @file    cmd_help.c
 @author  Netanel (PACABOT)
 @date    07/05/2015
 @version 1.0
 */
/**************************************************************************/

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "middleware/cmdline/commands/commads.h"
#include "middleware/cmdline/cmdline_parser.h"

int cmd_help(const char *args)
{
    UNUSED(args);

    CMD_HANDLER *hcmd = (CMD_HANDLER *) cmd_handlers;

    // Check if Display Callback was registered
    if (cmdline_ctxt.out == NULL)
    {
        return COMMAND_E_NO_OUTPUT;
    }

    cmdline_ctxt.out("\rAvailable commands:\r\r");

    while (hcmd->command != NULL)
    {
        cmdline_ctxt.out("%s        ", hcmd->command);
        if (hcmd->description != NULL)
        {
            cmdline_ctxt.out("%s\r", hcmd->description);
        }
        hcmd++;
    }

    return COMMAND_E_SUCCESS;
}
