/**************************************************************************/
/*!
    @file     expander.h
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for expander PCF8574
 */
/**************************************************************************/
#ifndef __PID_H__
#define __PID_H__

#include <arm_math.h>

typedef struct
{
	float32_t error_val;
	float32_t get_correction;
	arm_pid_instance_f32 * pid_instance;
} CONTROL_DEF;

void Pid_Init(arm_pid_instance_f32 * instance);
void Pid_Reset(arm_pid_instance_f32 * instance);
float32_t Pid(arm_pid_instance_f32 * instance, float32_t error);
void Pids_IT(void);

#endif
