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

void pidInit(arm_pid_instance_f32 * instance);
void pidReset(arm_pid_instance_f32 * instance);
float32_t pid(arm_pid_instance_f32 * instance, float32_t error);
void pids_IT(void);

#endif
