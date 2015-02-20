/**************************************************************************/
/*!
    @file     expander.h
    @author  PLF (PACABOT)
    @date
    @version  0.0

    Driver for expander PCF8574
 */
/**************************************************************************/
#ifndef __STRAIGHT_CONTROL_H__
#define __STRAIGHT_CONTROL_H__

typedef enum TypeOfSensors TypeOfSensors;
enum TypeOfSensors
{
    GYRO, ENCODERS, TELEMETERS
};

struct control
{
	char start_state;
	char distance;
};

void straightControlInit(TypeOfSensors Sensor_x);
void straightControl_IT(void);
void straightControlTest(void);
void straightControlTest(void);

#endif
