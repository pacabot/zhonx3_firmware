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

void Straight_Control_Start(TypeOfSensors Sensor_x);
void Straight_Control_IT(void);
void Debug_Straight_Control(void);

#endif
