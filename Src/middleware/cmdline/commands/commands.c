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
#include "stm32f4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

// Declarations for this module
#include "middleware/cmdline/cmdline_parser.h"
#include "middleware/cmdline/commands/commads.h"

/* Array of registered commands */
const CMD_HANDLER cmd_handlers[] =
        { { "help", cmd_help, "Displays the available commands" }, { "maze", cmd_maze, "Solve maze" }, { "test_distantce", NULL, "Gets front distance in millimeters" }, { "test_wall_sensor",
        NULL, "Tests wall sensors" }, { "test_bluetooth", NULL, "Tests Bluetooth connection" }, { "test_multimeter",
        NULL, "Tests Battery level" }, { "test_display", NULL, "Tests the SSD1306" }, { "test_eeprom",
        NULL, "Tests the EEPROM" }, { "test_encoders", NULL, "Tests motor encoders" }, { "test_joystick",
        NULL, "Tests User Interface Joystick" }, { "test_gyroscope",
        NULL, "Tests Gyroscope" }, { "test_telemeters", NULL, "Gets Telemeters ADC values" }, { "test_beeper",
        NULL, "Tests beeper" }, { "test_motors", NULL, "Tests robot motors" }, { "test_line_sensors",
        NULL, "Tests line sensors" }, { "halt", cmd_halt, "Shutdown Zhonx III" },

        { NULL, NULL, NULL } // This element is mandatory. It indicates the end of the array
        };

void cmd_output(const char *format, ...)
{
    UNUSED(format);

    return;
}

void cmd_displayPrompt(void)
{
    cmdline_ctxt.out("\rZhonxIII> ");
}
