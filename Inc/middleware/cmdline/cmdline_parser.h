/**************************************************************************/
/*!
    @file    cmdline_parser.h
    @author  Netanel (PACABOT)
    @date    02/05/2015
    @version 0.1
 */
/**************************************************************************/
#ifndef __CMDLINE_PARSER_H__
#define __CMDLINE_PARSER_H__

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define CMDLINE_PARSER_E_SUCCESS        0
/* Unspecified error */
#define CMDLINE_PARSER_E_ERROR          MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 1)
/* Command does not exist */
#define CMDLINE_PARSER_E_UNKNOWN_CMD    MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 2)
/* No callback registered for the given command */
#define CMDLINE_PARSER_E_NOT_REGISTERED MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 3)
/* No command found into command line */
#define CMDLINE_PARSER_E_CMD_NOT_FOUND  MAKE_ERROR(CMDLINE_PARSER_MODULE_ID, 4)

/* Commandline context */
typedef struct
{
    /* Callback function for output messages */
    void (*out)(const char *format, ...);
    /* Flag that indicates if a command has been received */
    int cmd_received;
    /* Pointer to Command line */
    char *cmdline;
    /* Length of received command */
    int cmd_len;
} CMDLINE_CONTEXT;

extern CMDLINE_CONTEXT cmdline_ctxt;


/**
 * @brief   Initializes the command line parser
 *
 * @retval  CMDLINE_PARSER_E_SUCCESS if operation is successful,
 *          a negative value otherwise
 */
int cmdline_init(CMDLINE_CONTEXT *context);

/**
 * @brief   Parses and executes a command line
 *
 * @retval  CMDLINE_PARSER_E_SUCCESS if operation is successful,
 *          a negative value otherwise
 */
int cmdline_parse(void);


#endif // __CMDLINE_PARSER_H__
