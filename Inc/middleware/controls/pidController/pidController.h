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
	arm_pid_instance_f32 * instance;
} pid_control_struct;

typedef struct
{
	char start_state;
}pid_loop_struct;

extern pid_control_struct pid_control;
extern pid_loop_struct pid_loop;

void pidControllerInit(arm_pid_instance_f32 * instance);
void pidControllerReset(arm_pid_instance_f32 * instance);
float32_t pidController(arm_pid_instance_f32 * instance, float32_t error);

#endif
