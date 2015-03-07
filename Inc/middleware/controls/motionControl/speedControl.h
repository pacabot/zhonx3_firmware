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
    uint8_t error_control;
	int16_t old_speed2;
    int16_t current_speed2;
    int16_t maintain_speed;
    uint32_t total_cnt;
    uint32_t old_cnt;
    uint32_t current_cnt;
    uint32_t maintain_cnt;
    uint32_t step_distance;
    float mm_distance;
}speed_control_struct;

int speedControl_Init(void);
int speedControl(void);
int speedAcc(uint32_t initial_speed, uint32_t distance);
int speedDcc(uint32_t final_speed, uint32_t distance);
int speedMaintain(uint32_t distance);

#endif
