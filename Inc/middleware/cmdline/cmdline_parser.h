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

/* Control keys */

/* CTRL+C */
#define CMDLINE_CTRL_C  0x03
/* CTRL+D */
#define CMDLINE_CTRL_D  0x04
/* Backspace */
#define CMDLINE_BS      0x08
/* TAB */
#define CMDLINE_TAB     0x09
/* Line feed */
#define CMDLINE_LF      0x0A
/* Carriage return */
#define CMDLINE_CR      0x0D

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
    /* Flag that indicates whether context is initialized or not */
    int is_initialized;
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
