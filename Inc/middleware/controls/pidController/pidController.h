/**************************************************************************/
/*!
    @file    pidController.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __PIDCONTROLLER_H__
#define __PIDCONTROLLER_H__

#include <arm_math.h>

typedef struct
{
	float32_t error_val;
	float32_t get_correction;
	arm_pid_instance_f32 * pid_instance;
} CONTROL_DEF;

void pidControllerInit(arm_pid_instance_f32 * instance);
void pidControllerReset(arm_pid_instance_f32 * instance);
float32_t pidController(arm_pid_instance_f32 * instance, float32_t error);
void pidController_IT(void);

#endif
