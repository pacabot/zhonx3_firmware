/**************************************************************************/
/*!
 @file    cmd_halt.c
 @author  Netanel (PACABOT)
 @date    29/05/2015
 @version 1.0
 */
/**************************************************************************/

/* General declarations */
#include <stm32f4xx_hal.h>
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "peripherals/display/ssd1306.h"

#include "middleware/settings/settings.h"
#include "middleware/cmdline/commands/commads.h"
#include "middleware/cmdline/cmdline_parser.h"

#include "stm32f4xx_hal.h"

int cmd_halt(const char *args)
{
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306Printf(0, 0, &Font_8x8, "GOOD BYE!!!");
    ssd1306Printf(0, 15, &Font_8x8, args);
    ssd1306Refresh();

    HAL_Delay(5000);

    cmdline_ctxt.out("\nGOOD BYE!!!");
    halt();
    return COMMAND_E_SUCCESS;
}
