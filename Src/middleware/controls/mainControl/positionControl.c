/**************************************************************************/
/*!
 @file    positionControl.c
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
/* STM32 hal library declarations */
#include "stm32f4xx_hal.h"

/* General declarations */
#include "config/basetypes.h"
#include "config/config.h"
#include "config/errors.h"

#include "stdbool.h"
#include <arm_math.h>
#include <math.h>
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/speedControl.h>
#include <middleware/controls/mainControl/transfertFunction.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/telemeters/telemeters.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/gyroscope/adxrs620.h"
#include "peripherals/motors/motors.h"
#include "peripherals/times_base/times_base.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"

#include <middleware/controls/mainControl/positionControl.h>

#define MAX_POSITION_ERROR     20.00 //Degrees

typedef struct
{
    int nb_loop_accel;  //rotate in place
    int nb_loop_maint;
    int nb_loop_decel;
    double accel_angle_per_loop;    //rotate in place

    double angle_consign;	    //total angle
    double angle_per_loop;
    double max_speed;
    int nb_loop;
    int sign;
} position_params_struct;

typedef struct
{
    int nb_loop_accel;  //rotate in place
    int nb_loop_maint;
    int nb_loop_decel;

    int nb_loop;
    double position_command;
    double position_error;
    double position_consign;
    double current_angle;
    double current_angle_consign;   //differential distance (mm) since the control start
    double old_angle;				//effective distance at the previous call
    char end_control;
    enum enablePositionCtrl enablePositionCtrl;
    enum positionType positionType;
    pid_control_struct position_pid;
} position_control_struct;

/* App definitions */

/* Macros */

/* Static functions */

/* extern variables */

/* global variables */
static position_control_struct position_control;
static position_params_struct position_params;
static arm_pid_instance_f32 gyro_pid_instance;

int positionControlInit(void)
{
    memset(&position_control, 0, sizeof(position_control_struct));
    memset(&position_params, 0, sizeof(position_params_struct));
    positionProfileCompute(0, 0, 0);

    gyro_pid_instance.Kp = 90;
    gyro_pid_instance.Ki = 0;
    gyro_pid_instance.Kd = 1500;

//    gyro_pid_instance.Kp = zhonxCalib_data->pid_gyro.Kp;
//    gyro_pid_instance.Ki = zhonxCalib_data->pid_gyro.Ki / CONTROL_TIME_FREQ;
//    gyro_pid_instance.Kd = zhonxCalib_data->pid_gyro.Kd * CONTROL_TIME_FREQ;

    position_control.position_pid.instance = &gyro_pid_instance;

    position_control.positionType = GYRO;
    position_params.sign = 1;

    pidControllerInit(position_control.position_pid.instance);

    return POSITION_CONTROL_E_SUCCESS;
}

char positionControlHasMoveEnded(void)
{
    return position_control.end_control;
}

double positionControlGetCurrentAngle(void)
{
    return position_control.current_angle;
}

double positionControlGetPositionCommand(void)
{
    return position_control.position_command;
}

char positionControlSetPositionType(enum positionType position_type)
{
    position_control.positionType = position_type;
    return POSITION_CONTROL_E_SUCCESS;
}

char positionControlEnablePositionCtrl(enum enablePositionCtrl enable_position_ctrl)
{
    position_control.enablePositionCtrl = enable_position_ctrl;
    return POSITION_CONTROL_E_SUCCESS;
}

double positionControlSetSign(double sign)
{
    return position_params.sign = sign;
}

int positionControlLoop(void)
{
    //    if (mainControlGetWallFollowType() != CURVE)
    //    {
    //        if (position_control.enablePositionCtrl == NO_POSITION_CTRL)
    //        {
    //            position_control.position_command = 0;
    //            pidControllerReset(position_control.position_pid.instance);
    //            return POSITION_CONTROL_E_SUCCESS;
    //        }
    //    }

//    if (mainControlGetWallFollowType() == ROTATE_IN_PLACE)
//    {
//        if (position_control.nb_loop_accel < position_params.nb_loop_accel)
//        {
//            position_control.nb_loop_accel++;
//            position_control.position_consign += position_params.accel_angle_per_loop;
//            position_control.current_angle_consign += position_control.position_consign;
//        }
//        else if (position_control.nb_loop_maint < position_params.nb_loop_maint)
//        {
//            position_control.nb_loop_maint++;
//            position_control.current_angle_consign += position_params.angle_per_loop;
//        }
//        else if (position_control.nb_loop_decel < position_params.nb_loop_decel)
//        {
//            position_control.nb_loop_decel++;
//            position_control.position_consign -= position_params.accel_angle_per_loop;
//            position_control.current_angle_consign += position_params.angle_per_loop;
//        }
//        else
//        {
//            position_control.end_control = TRUE;
//        }
//    }
//    else
    {

        if (position_control.positionType == GYRO)
        {
            if (position_params.sign > 0)
                position_control.current_angle = gyroGetAngle();
            else
                position_control.current_angle = -1.00 * gyroGetAngle();
        }
//        else //use encoders
//        {
//            ledPowerErrorBlink(300, 300, 2);
//            if (position_params.sign > 0)
//                position_control.current_angle = 180.00 * (encoderGetDist(ENCODER_L) - encoderGetDist(ENCODER_R))
//                / (PI * ROTATION_DIAMETER);
//            else
//                position_control.current_angle = 180.00 * (encoderGetDist(ENCODER_R) - encoderGetDist(ENCODER_L))
//                / (PI * ROTATION_DIAMETER);
//        }

        if (position_control.nb_loop < position_params.nb_loop)
        {
            position_control.nb_loop++;
            position_control.current_angle_consign += position_params.angle_per_loop;
        }
        else
        {
            position_control.end_control = TRUE;
        }
    }

    position_control.position_error = position_control.current_angle_consign - position_control.current_angle;//for distance control
    if (fabs(position_control.position_error) > MAX_POSITION_ERROR)
    {
        ledPowerErrorBlink(1000, 150, 5);
        telemetersStop();
        motorsDriverSleep(ON);
        return POSITION_CONTROL_E_ERROR;
    }

    position_control.position_command = (pidController(position_control.position_pid.instance,
                                                       position_control.position_error)) * (float) position_params.sign;

    position_control.old_angle = position_control.current_angle;

    return POSITION_CONTROL_E_SUCCESS;
}

/**************************************************************************/
/*!
 ***BASICS FORMULAS***

       ___   _________
      / 2 x / Acc x d
 t = v_____v__________	//without initial speed
            Acc

             __________________
     - Vi + / Vi²+ 2 x Acc x d
 t = ______v___________________	//with initial speed
                 Acc

         V²
 d =   -----
       2.Acc

      1
 d = --- Acc x t²
      2

               1
 d = Vi x t + --- Acc x t²
               2

 Vf = Vi + Acc x t 	//instantaneous speed

     Vi + Vf
 V = -------			//average speed
        2

        2.d
 Acc = -----
         t²

 d = V x t

 V = Vi + (Acc x t)
                                   ________________________
      / 1 \             / 1 \	  /
 v =  |---| x t x Acc + |---| x  V t² x Acc² - 4 x Acc x d	//v = f(t,d,Acc)
      \ 2 /			    \ 2 /

        -2(t.Vi-d)
 Acc = ------------
            t²

         2(t.Vi-d)
 Dcc = ------------
             t²
 */
/**************************************************************************/
double positionProfileCompute(double angle, double loop_time, double max_turn_speed)
{
    position_control.nb_loop_accel = 0;  //rotate in place
    position_control.nb_loop_maint = 0;
    position_control.nb_loop_decel = 0;

    position_control.nb_loop = 0;
    position_control.position_command = 0;
    position_control.position_error = 0;
    position_control.current_angle = 0;
    position_control.current_angle_consign = 0;
    position_control.position_consign = 0;
    position_control.end_control = FALSE;

    if (lround(angle) == 0)
    {
        position_control.end_control = TRUE;
        position_params.nb_loop = 0;
        position_params.angle_consign = 0;
        return (0);
    }
    if (lround(loop_time) == 0)
    {
        loop_time = (angle / max_turn_speed) * CONTROL_TIME_FREQ;
    }

    position_params.nb_loop = (int)loop_time;
    position_params.angle_per_loop = angle / (double)position_params.nb_loop;

    return (position_params.nb_loop);
}
