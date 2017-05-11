/*
 * safety_stop.h
 *
 *  Created on: 10 mai 2017
 *      Author: zhonx
 */

#ifndef INC_MIDDLEWARE_SAFETY_STOP_H_
#define INC_MIDDLEWARE_SAFETY_STOP_H_

/* Module Identifier */
#include "config/module_id.h"

/* Error codes */
#define SAFETY_STOP_E_SUCCESS  0
#define SAFETY_STOP_E_ERROR    MAKE_ERROR(SAFETY_STOP_MODULE_ID, 1)


#define GPIO_PW_KILL_PIN       GPIO_PIN_8
#define GPIO_PW_KILL_PORT      GPIOB

void emergencyStop(void);
void halt(void);

#endif /* INC_MIDDLEWARE_SAFETY_STOP_H_ */
