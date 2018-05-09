/**************************************************************************/
/*!
 @file    speedControl.c
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
#include <middleware/controls/mainControl/positionControl.h>
#include <middleware/controls/mainControl/speedControl.h>
#include <middleware/controls/mainControl/speedControl.h>
#include <middleware/controls/mainControl/transfertFunction.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Peripheral declarations */
#include "peripherals/display/ssd1306.h"
#include "peripherals/display/smallfonts.h"
#include "peripherals/expander/pcf8574.h"
#include "peripherals/encoders/ie512.h"
#include "peripherals/motors/motors.h"
#include "peripherals/tone/tone.h"
#include "peripherals/bluetooth/bluetooth.h"
#include "peripherals/times_base/times_base.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"

#define MAX_SPEED_ERROR     120.00 //Millimeter //todo gros bug de merde 113
#define SPEED_ERROR_LIMITER 10.00 //Millimeter

typedef struct
{
    double distance_consign;			//total distance
    double max_speed;
    double initial_speed;
    double end_speed;
    double accel;
    double decel;
    double accel_dist;
    double decel_dist;
    double accel_dist_per_loop;
    double decel_dist_per_loop;
    int nb_loop_accel;
    int nb_loop_decel;
    int nb_loop_maint;
    double maintain_dist;
    double sign;
} speed_params_struct;

//extern speed_params_struct speed_params;

typedef struct
{
    int nb_loop_accel;
    int nb_loop_maint;
    int nb_loop_decel;
    double current_distance;			//distance (mm) since the control start
    double gap_distance_per_loop;	//distance between two control loop call
    double current_distance_consign;		//distance consign for current loop
    double old_distance;			//effective distance at the previous call
    double current_speed;
    double speed_consign;
    double speed_error;
    double speed_command;
    char end_control;

    pid_control_struct speed_pid;
} speed_control_struct;

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */

/* private variables */
static speed_params_struct speed_params;
static speed_control_struct speed_control;
static arm_pid_instance_f32 encoder_pid_instance;

int speedControlInit(void)
{
    memset(&speed_control, 0, sizeof(speed_control_struct));
    memset(&speed_params, 0, sizeof(speed_params_struct));
    speedProfileCompute(0, 0, 0, 0);

    encoder_pid_instance.Kp = 700;//638.00;
    encoder_pid_instance.Ki = 0;//0.02666 / CONTROL_TIME_FREQ;
    encoder_pid_instance.Kd = 2 * CONTROL_TIME_FREQ;

    speed_control.speed_pid.instance = &encoder_pid_instance;

    pidControllerInit(speed_control.speed_pid.instance);

    return SPEED_CONTROL_E_SUCCESS;
}

char speedControlHasMoveEnded(void)
{
    return speed_control.end_control;
}

double speedControlGetCurrentDist(void)
{
    return speed_control.current_distance;
}

double speedControlGetSpeedCommand(void)
{
    return speed_control.speed_command;
}

double speedControlSetSign(double sign)
{
    return speed_params.sign = sign;
}

int speedControlLoop(void)
{
    if (speed_params.sign > 0)
        speed_control.current_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;
    else
        speed_control.current_distance = -1.00 * (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;

    //	speedCompute();

    if (speed_control.nb_loop_accel < speed_params.nb_loop_accel)
    {
        speed_control.nb_loop_accel++;
        speed_control.speed_consign += speed_params.accel_dist_per_loop;
        speed_control.current_distance_consign += speed_control.speed_consign;
    }
    else if (speed_control.nb_loop_maint < speed_params.nb_loop_maint)
    {
        speed_control.nb_loop_maint++;
        speed_control.current_distance_consign += speed_control.speed_consign;
    }
    else if (speed_control.nb_loop_decel < speed_params.nb_loop_decel)
    {
        speed_control.nb_loop_decel++;
        speed_control.speed_consign -= speed_params.decel_dist_per_loop;
        speed_control.current_distance_consign += speed_control.speed_consign;
    }
    else
    {
        speed_control.end_control = TRUE;
    }

    speed_control.speed_error = speed_control.current_distance_consign - speed_control.current_distance;//for distance control
    if (fabs(speed_control.speed_error) > MAX_SPEED_ERROR)
    {
        ledPowerErrorBlink(1000, 150, 3);
        return SPEED_CONTROL_E_ERROR;
    }
    if (fabs(speed_control.speed_error) > SPEED_ERROR_LIMITER)
    {
        if (speed_control.speed_error > 0)
            speed_control.speed_error = SPEED_ERROR_LIMITER;
        else
            speed_control.speed_error = -SPEED_ERROR_LIMITER;
    }

    speed_control.speed_command = pidController(speed_control.speed_pid.instance, speed_control.speed_error)
            * (float) speed_params.sign;

    //bluetoothPrintf("speed error: %d \r\n", (int)(speed_control.speed_error * 100.00));

    speed_control.old_distance = speed_control.current_distance;

    return SPEED_CONTROL_E_SUCCESS;
}

int speedCompute(void)
{
    speed_control.gap_distance_per_loop = speed_control.current_distance - speed_control.old_distance;//delta distance per loop
    speed_control.current_speed = (speed_control.gap_distance_per_loop * CONTROL_TIME_FREQ);			//actual speed (mm/s)

    return SPEED_CONTROL_E_SUCCESS;
}

/**************************************************************************/
/*!
 ***BASICS FORMULAS***

       ___   _________
      / 2 x / Acc x d
 t = v_____v__________  //without initial speed
            Acc

             __________________
     - Vi + / Vi²+ 2 x Acc x d
 t = ______v___________________ //with initial speed
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

 Vf = Vi + Acc x t  //instantaneous speed

     Vi + Vf
 V = -------            //average speed
        2

        2.d
 Acc = -----
         t²

 d = V x t

 V = Vi + (Acc x t)
                                   ________________________
      / 1 \             / 1 \     /
 v =  |---| x t x Acc + |---| x  V t² x Acc² - 4 x Acc x d  //v = f(t,d,Acc)
      \ 2 /             \ 2 /

        -2(t.Vi-d)
 Acc = ------------
            t²

         2(t.Vi-d)
 Dcc = ------------
             t²

 */
/**************************************************************************/
double speedProfileCompute(double distance, double max_speed, double end_speed, double accel)
{
#ifdef DEBUG_SPEED_CONTROL
    char str[50];
#endif
    speed_params.accel = accel;
    speed_params.decel = accel;

    speed_control.nb_loop_accel = 0;
    speed_control.nb_loop_maint = 0;
    speed_control.nb_loop_decel = 0;
    speed_control.current_distance = 0;
    speed_control.gap_distance_per_loop = 0;
    speed_control.current_distance_consign = 0;
    speed_control.old_distance = 0;
    speed_control.current_speed = 0;
    speed_control.speed_error = 0;
    speed_control.speed_command = 0;
    speed_control.speed_consign = 0;
    speed_control.end_control = FALSE;

    pidControllerReset(speed_control.speed_pid.instance);

    if (lround(distance) == 0)
    {
        speed_control.end_control = TRUE;
//        speed_params.initial_speed = 0;
//        speed_control.end_control = TRUE;
//        speed_params.nb_loop_accel = 0;
//        speed_params.nb_loop_decel = 0;
//        speed_params.nb_loop_maint = 0;
//        speed_params.end_speed = 0;
//        speed_params.distance_consign = 0;

        speed_params.distance_consign = 0;
        speed_params.max_speed = 0;
        speed_params.initial_speed = 0;
        speed_params.end_speed = 0;
        speed_params.accel = 0;
        speed_params.decel = 0;
        speed_params.accel_dist = 0;
        speed_params.decel_dist = 0;
        speed_params.accel_dist_per_loop = 0;
        speed_params.decel_dist_per_loop = 0;
        speed_params.nb_loop_accel = 0;
        speed_params.nb_loop_decel = 0;
        speed_params.nb_loop_maint = 0;
        speed_params.maintain_dist = 0;
        speed_params.sign = 0;
        return 0.0;
    }

    if ((end_speed > max_speed) || (lround(end_speed) == lround(max_speed)))
    {
        speed_params.max_speed = end_speed;
        speed_params.end_speed = end_speed;
        speed_params.accel_dist = (((pow(speed_params.max_speed, 2) - pow(speed_params.initial_speed, 2))) / ( 2.00 * speed_params.accel));
        speed_params.decel_dist = 0;

        if (speed_params.accel_dist >= distance)
        {
            speed_params.accel_dist = distance;
            speed_params.decel_dist = 0.00;
        }
    }
    else if (end_speed < max_speed)
    {
        speed_params.end_speed = end_speed;
        speed_params.max_speed = max_speed;
        speed_params.accel_dist = (((pow(speed_params.max_speed, 2) - pow(speed_params.initial_speed, 2))) / ( 2.00 * speed_params.accel));
        speed_params.decel_dist = (((pow(speed_params.max_speed, 2) - pow(speed_params.end_speed, 2))) / ( 2.00 * speed_params.accel));

        if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
        {
            //        double clipping_ratio;
            //        clipping_ratio = (distance / (speed_params.accel_dist + speed_params.decel_dist));
            //        speed_params.accel_dist *= clipping_ratio;
            //        speed_params.decel_dist *= clipping_ratio;
            //        speed_params.max_speed = sqrt(pow(speed_params.initial_speed, 2) + 2.00 * speed_params.accel * speed_params.accel_dist);

            speed_params.max_speed = sqrt((2.00 * speed_params.accel * distance + pow(end_speed, 2) + pow(speed_params.initial_speed, 2))/2.00);

            speed_params.accel_dist = 0.50 * (((-1.00 * pow(speed_params.initial_speed, 2)) +
                    pow(speed_params.max_speed, 2)) / speed_params.accel);
            speed_params.decel_dist = -0.50 * ((speed_params.end_speed - speed_params.max_speed) *
                    (speed_params.end_speed + speed_params.max_speed)) / speed_params.decel;
        }
    }
    //    else if (end_speed < max_speed)
    //    {
    //        speed_params.end_speed = end_speed;
    //        speed_params.max_speed = max_speed;
    //        speed_params.accel_dist = 0;
    //        speed_params.decel_dist = (((pow(speed_params.max_speed, 2) - pow(speed_params.end_speed, 2))) / ( 2.00 * speed_params.accel));
    //
    //        if (speed_params.decel_dist >= distance)
    //        {
    //            speed_params.accel_dist = 0;
    //            speed_params.decel_dist = distance;
    //        }
    //    }
    else
    {
        return 0.00;
    }

    //    speed_params.accel_dist = (((pow(speed_params.max_speed, 2) - pow(speed_params.initial_speed, 2))) / ( 2.00 * speed_params.accel));
    //
    ////    speed_params.accel_dist = 0.50 * (((-1.00 * pow(speed_params.initial_speed, 2)) +
    ////            pow(speed_params.max_speed, 2)) / speed_params.accel);
    //    speed_params.decel_dist = -0.50 * ((speed_params.end_speed - speed_params.max_speed) *
    //            (speed_params.end_speed + speed_params.max_speed)) / speed_params.decel;

    speed_params.accel_dist_per_loop = speed_params.accel / pow(CONTROL_TIME_FREQ, 2);
    speed_params.decel_dist_per_loop = speed_params.decel / pow(CONTROL_TIME_FREQ, 2);

    speed_control.speed_consign = (speed_params.initial_speed / CONTROL_TIME_FREQ);


    speed_params.nb_loop_accel = (((-1.00 * speed_params.initial_speed)
            + sqrt(pow(speed_params.initial_speed, 2) + 2.00 * speed_params.accel * speed_params.accel_dist))
            / speed_params.accel) * CONTROL_TIME_FREQ;

    speed_params.nb_loop_decel = (((speed_params.max_speed)
            - sqrt(pow(speed_params.max_speed, 2) - 2.00 * speed_params.decel * speed_params.decel_dist))
            / speed_params.decel) * CONTROL_TIME_FREQ;

    if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
    {
        speed_params.maintain_dist = 0;
        speed_params.nb_loop_maint = 0;
    }
    else
    {
        speed_params.maintain_dist = distance - (speed_params.accel_dist + speed_params.decel_dist);
        speed_params.nb_loop_maint = ((speed_params.maintain_dist / (speed_params.initial_speed +
                (speed_params.accel * ((double) speed_params.nb_loop_accel / CONTROL_TIME_FREQ)))) * //compute Speed
                CONTROL_TIME_FREQ);
    }

#ifdef DEBUG_SPEED_CONTROL
    bluetoothWaitReady();
    bluetoothPrintf("Dist_tot = %d, V_init = %d, AccTime = %d, MainTime = %d, DccTime = %d, AccDist = %d, MainDist = %d, DccDist = %d, Acc = %d, Dcc = %d\r",
                    (int)distance,
                    (int)speed_params.initial_speed,
                    speed_params.nb_loop_accel,
                    speed_params.nb_loop_maint,
                    speed_params.nb_loop_decel,
                    (int)speed_params.accel_dist,
                    (int)speed_params.maintain_dist,
                    (int)speed_params.decel_dist,
                    (int)speed_params.accel_dist_per_loop,
                    (int)speed_params.decel_dist_per_loop);
#endif

    speed_params.initial_speed = speed_params.initial_speed
            + (((speed_params.nb_loop_accel / CONTROL_TIME_FREQ) * speed_params.accel)
                    - ((speed_params.nb_loop_decel / CONTROL_TIME_FREQ) * speed_params.decel));

    speed_params.distance_consign = distance;

    double move_loop_time = (speed_params.nb_loop_accel + speed_params.nb_loop_decel + speed_params.nb_loop_maint);
    //  bluetoothPrintf("nombre de deplacement: %d,nb_loop_accel = %d, nb_loop_decel = %d, nb_loop_maint = %d \r\n", i, (int)speed_params.nb_loop_accel, (int)speed_params.nb_loop_decel, (int)speed_params.nb_loop_maint);
    speed_control.end_control = FALSE;
    return move_loop_time;
//    char str[50];
//    speed_params.end_speed = end_speed;
//    speed_params.max_speed = max_speed;
//    speed_params.accel = accel;
//    speed_params.decel = accel;
//
//    speed_control.nb_loop_accel = 0;
//    speed_control.nb_loop_maint = 0;
//    speed_control.nb_loop_decel = 0;
//    speed_control.current_distance = 0;
//    speed_control.gap_distance_per_loop = 0;
//    speed_control.current_distance_consign = 0;
//    speed_control.old_distance = 0;
//    speed_control.current_speed = 0;
//    speed_control.end_control = FALSE;
//    speed_control.speed_error = 0;
//    speed_control.speed_command = 0;
//    speed_control.speed_consign = 0;
//
//    pidControllerReset(speed_control.speed_pid.instance);
//
//    if (lround(distance) == 0)
//    {
//        speed_params.initial_speed = 0;
//        speed_control.end_control = TRUE;
//        speed_params.nb_loop_accel = 0;
//        speed_params.nb_loop_decel = 0;
//        speed_params.nb_loop_maint = 0;
//        speed_params.end_speed = 0;
//        speed_params.distance_consign = 0;
//        return 0.0;
//    }
//
//    speed_params.accel_dist = 0.50
//            * (((-1.00 * pow(speed_params.initial_speed, 2)) + pow(speed_params.max_speed, 2)) / speed_params.accel);
//    speed_params.decel_dist = -0.50
//            * ((speed_params.end_speed - speed_params.max_speed) * (speed_params.end_speed + speed_params.max_speed))
//            / speed_params.decel;
//
//    speed_params.accel_dist_per_loop = speed_params.accel / pow(CONTROL_TIME_FREQ, 2);
//    speed_params.decel_dist_per_loop = speed_params.decel / pow(CONTROL_TIME_FREQ, 2);
//
//    speed_control.speed_consign = (speed_params.initial_speed / CONTROL_TIME_FREQ);
//    speed_control.current_distance_consign = 0.00;
//    speed_control.end_control = 0;
//
//    if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
//    {
//        double clipping_ratio;
//        clipping_ratio = (distance / (speed_params.accel_dist + speed_params.decel_dist));
//        speed_params.accel_dist *= clipping_ratio;
//        speed_params.decel_dist *= clipping_ratio;
//        speed_params.max_speed = sqrt(
//                pow(speed_params.initial_speed, 2) + 2.00 * speed_params.accel * speed_params.accel_dist);
//    }
//
//    speed_params.nb_loop_accel = (((-1.00 * speed_params.initial_speed)
//            + sqrt(pow(speed_params.initial_speed, 2) + 2.00 * speed_params.accel * speed_params.accel_dist))
//            / speed_params.accel) * CONTROL_TIME_FREQ;
//    speed_params.nb_loop_decel = (((speed_params.max_speed)
//            - sqrt(pow(speed_params.max_speed, 2) - 2.00 * speed_params.decel * speed_params.decel_dist))
//            / speed_params.decel) * CONTROL_TIME_FREQ;
//
//    if ((speed_params.accel_dist + speed_params.decel_dist) > distance)
//    {
//        speed_params.maintain_dist = 0;
//        speed_params.nb_loop_maint = 0;
//    }
//    else
//    {
//        speed_params.maintain_dist = distance - (speed_params.accel_dist + speed_params.decel_dist);
//        speed_params.nb_loop_maint = ((speed_params.maintain_dist
//                / (speed_params.initial_speed
//                        + (speed_params.accel * ((double) speed_params.nb_loop_accel / CONTROL_TIME_FREQ)))) * //compute Speed
//                CONTROL_TIME_FREQ);
//    }
//
//    speed_params.initial_speed = speed_params.initial_speed
//            + (((speed_params.nb_loop_accel / CONTROL_TIME_FREQ) * speed_params.accel)
//                    - ((speed_params.nb_loop_decel / CONTROL_TIME_FREQ) * speed_params.decel));
//
//    speed_params.distance_consign = distance;
//
//    double move_loop_time = (speed_params.nb_loop_accel + speed_params.nb_loop_decel + speed_params.nb_loop_maint);
//    //	bluetoothPrintf("nombre de deplacement: %d,nb_loop_accel = %d, nb_loop_decel = %d, nb_loop_maint = %d \r\n", i, (int)speed_params.nb_loop_accel, (int)speed_params.nb_loop_decel, (int)speed_params.nb_loop_maint);
//    return move_loop_time;
}
