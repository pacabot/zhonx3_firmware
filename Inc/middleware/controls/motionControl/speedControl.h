/**************************************************************************/
/*!
    @file    speedControl.h
    @author  PLF (PACABOT)
    @date
    @version  0.0
 */
/**************************************************************************/
#ifndef __SPEEDCONTROL_H__
#define __SPEEDCONTROL_H__

/* Module Identifier */
#define SPEED_CONTROL_MODULE_ID  100

/* Error codes */
#define SPEED_CONTROL_E_SUCCESS  0
#define SPEED_CONTROL_E_ERROR    MAKE_ERROR(SPEED_CONTROL_MODULE_ID, 1)

/* Types definitions */
typedef struct
{
	float speed_consigne;
    float error_control;
	float old_speed2;
	float old_speed;
    float current_speed;
    float current_speed2;
    int16_t maintain_speed;
    float total_cnt;
    float old_cnt;
    float current_cnt;
    uint32_t maintain_cnt;
    float step_distance;
    float mm_distance;
    CONTROL_DEF speed;
}speed_control_struct;

int speedControl_Init(void);
int speedControl(void);
int speedAcc(uint32_t initial_speed, uint32_t distance);
int speedDcc(uint32_t final_speed, uint32_t distance);
int speedMaintain(uint32_t distance);

#endif
