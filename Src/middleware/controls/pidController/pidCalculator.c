/**************************************************************************/
/*!
 @file    pidCalculator.c
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
#include <middleware/settings/settings.h>
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/positionControl.h>
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
#include "peripherals/gyroscope/adxrs620.h"

/* Middleware declarations */
#include "middleware/controls/pidController/pidController.h"

/* declaration for this module */
#include "middleware/controls/pidController/pidCalculator.h"

/* Macros */

/* Static functions */

/* extern variables */
/* global variables */

/* private variables */

void accelMotor_GetStepResponse(void)
{
    motorsInit();
    encodersInit();
    adxrs620Init();
    gyroResetAngle();

    HAL_Delay(1000);

    uint16_t buff[10000] = { 0 };
    double speed = 0.00;
    const int refresh_frequency = 1000;
    double current_distance = 0.00;
    double current_angle = 0.00;
    double old_distance = 0.00;
    double old_angle = 0.00;

    int i = 0;

    motorsInit();
    encodersInit();

    HAL_Delay(1000);

    // Forward Fast (PWM on IN1, LOW on IN2)
    motorSet_DF(MOTOR_L, 200);
    motorSet_DF(MOTOR_R, -200);
    motorsDriverSleep(OFF);

    while ((int) current_angle < 400) //((int)speed < 3000 && (int)current_distance < 4000)
    {
        current_angle = gyroGetAngle();

        speed = ((current_angle - old_angle) * (double) refresh_frequency); //actual speed (mm/s)

        buff[i] = (uint16_t) speed;
        i++;

        HAL_Delay(1000 / refresh_frequency);
        old_distance = current_angle;
    }

    motorsBrake();
    motorsDriverSleep(ON);

    bluetoothSend((uint8_t*) buff, i * 2);

    return;

    // Forward Fast (PWM on IN1, LOW on IN2)
    motorSet_DF(MOTOR_L, 100);
    motorSet_DF(MOTOR_R, 100);
    motorsDriverSleep(OFF);

    while ((int) current_distance < 400) //((int)speed < 3000 && (int)current_distance < 4000)
    {
        current_distance = (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00;

        speed = ((current_distance - old_distance) * (double) refresh_frequency); //actual speed (mm/s)

        buff[i] = (uint16_t) speed;
        i++;

        HAL_Delay(1000 / refresh_frequency);
        old_distance = current_distance;
    }

    motorsBrake();
    motorsDriverSleep(ON);

    bluetoothSend((uint8_t*) buff, i * 2);
}

void pidEncoder_GetCriticalPoint(void)
{
}
/*
 * Kcr => critical proportional gain
 * Tcr => time period
 * ______________________________________________________________
 * |     Control Type       |   K_p     |   K_i     |   K_d     |
 * |------------------------+-----------+-----------+-----------|
 * |           P            |   0.5Kcr  |     -     |     -     |
 * |------------------------+-----------+-----------+-----------|
 * |           PI           |   0.45Kcr |   Tcr/1.2 |     -     |
 * |------------------------+-----------+-----------+-----------|
 * |           PD           |   0.8Kcr  |     -     |   Tcr/8   |
 * |------------------------+-----------+-----------+-----------|
 * |      classic PID       |   0.60Kcr |   Tcr/2   |   Tcr/8   |
 * |------------------------+-----------+-----------+-----------|
 * | Pessen Integral Rule2  |   0.7Kcr  |   Tcr/2.5 |  03Tcr/20 |
 * |------------------------+-----------+-----------+-----------|
 * |     some overshoot     |   0.33Kcr |   Tcr/2   |   Tcr/3   |
 * |------------------------+-----------+-----------+-----------|
 * |     no overshoot       |   0.2Kcr  |   Tcr/2   |   Tcr/3   |
 * --------------------------------------------------------------
 *
 * ______________________________________________________________
 * |     Control Type       |   K_p     |   K_i     |   K_d     |
 * |------------------------+-----------+-----------+-----------|
 * |          P             |   0.5Kcr  |     -     |     -     |
 * |------------------------+-----------+-----------+-----------|
 * |          PI            |   0.45Kcr | 1.2Kp/Tcr |     -     |
 * |------------------------+-----------+-----------+-----------|
 * |          PID           |   0.6Kcr  |  2Kp/Tcr  | Kp*Tcr/8  |
 * --------------------------------------------------------------
 */
void pidGyro_GetCriticalPoint(void)
{
    int Kp_avrg_cnt = 0;
    int Kp_avrg = 0;
    int cnt = 0;
    int osc_cnt = 0;
    int period_time_ms = 0;
    int initial_time_ms = 0;
    char old_sign = 0;
    unsigned char rising_edge_counter = 0;
    unsigned int period_time_cnt = 0;
    double position_command = 0;
    arm_pid_instance_f32 position_pid;
    mobileAvrgStruct mAvrgStruct;
    memset((mobileAvrgStruct*) &mAvrgStruct, 0, sizeof(mobileAvrgStruct));
    const double pwm_ratio = (PWM_RATIO_COEFF_A * 8000.00 + PWM_RATIO_COEFF_B);
    const double error_max = 0.1; //angular error
    const int pwm_move_offset = 200;
    const int max_dist = 1000;
    position_pid.Kp = 100;
    position_pid.Ki = 0;
    position_pid.Kd = 3000;
    arm_pid_instance_f32 coefs;
    int rv;

    encodersInit();
    adxrs620Init();
    motorsInit();

    encodersReset();
    gyroResetAngle();

    pidControllerInit(&position_pid);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(0, 0, "         POSITION PID CAL", &Font_3x6);
        ssd1306DrawStringAtLine(0, 1, "  RIGHT TO START, LEFT TO EXIT", &Font_3x6);

        ssd1306PrintfAtLine(0, 2, &Font_5x8, "Kp = %d", (uint32_t)zhonxCalib_data->pid_gyro.Kp);
        ssd1306PrintfAtLine(0, 3, &Font_5x8, "Ki = %d", (uint32_t)(zhonxCalib_data->pid_gyro.Ki));
        ssd1306PrintfAtLine(0, 4, &Font_5x8, "Kd = %d.10^-3", (uint32_t)(zhonxCalib_data->pid_gyro.Kd * 1000));
        ssd1306Refresh();

        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(100);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();
    HAL_Delay(1000);
    motorsDriverSleep(OFF);

    while(expanderJoyFiltered() != JOY_LEFT)        //for manual testing
    {
        position_command = (pidController(&position_pid, gyroGetAngle()));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio));
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio));
        HAL_Delay(1);
    }
}

void pidEncoders_GetCriticalPoint(void)
{
    int Kp_avrg_cnt = 0;
    int Kp_avrg = 0;
    int cnt = 0;
    int osc_cnt = 0;
    int period_time_ms = 0;
    int initial_time_ms = 0;
    char old_sign = 0;
    unsigned char rising_edge_counter = 0;
    unsigned int period_time_cnt = 0;
    double speed_command = 0;
    arm_pid_instance_f32 speed_pid;
    mobileAvrgStruct mAvrgStruct;
    memset((mobileAvrgStruct*) &mAvrgStruct, 0, sizeof(mobileAvrgStruct));
    const double pwm_ratio = (PWM_RATIO_COEFF_A * 7400.00 + PWM_RATIO_COEFF_B);
    const double error_max = 0.1; //angular error
    const int pwm_move_offset = 200;
    const int max_dist = 1000;
    speed_pid.Kp = 500;
    speed_pid.Ki = 0;
    speed_pid.Kd = 2000;
    arm_pid_instance_f32 coefs;
    int rv;

    encodersInit();
    adxrs620Init();
    motorsInit();

    encodersReset();
    gyroResetAngle();

    pidControllerInit(&speed_pid);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(0, 0, "         SPEED PID CAL", &Font_3x6);
        ssd1306DrawStringAtLine(0, 1, "  RIGHT TO START, LEFT TO EXIT", &Font_3x6);

//        ssd1306PrintfAtLine(0, 2, &Font_5x8, "Kp = %d", (uint32_t)zhonxCalib_data->pid_gyro.Kp);
//        ssd1306PrintfAtLine(0, 3, &Font_5x8, "Ki = %d", (uint32_t)(zhonxCalib_data->pid_gyro.Ki));
//        ssd1306PrintfAtLine(0, 4, &Font_5x8, "Kd = %d.10^-3", (uint32_t)(zhonxCalib_data->pid_gyro.Kd * 1000));
        ssd1306Refresh();

        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(100);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();
    HAL_Delay(1000);
    motorsDriverSleep(OFF);

    while(expanderJoyFiltered() != JOY_LEFT)        //for manual testing
    {
        speed_command = (pidController(&speed_pid, (encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00));

        motorSet_DF(MOTOR_R, (int)(-speed_command * pwm_ratio));
        motorSet_DF(MOTOR_L, (int)(-speed_command * pwm_ratio));
        HAL_Delay(1);
    }
}

void pidTelemeters_GetCriticalPoint(void)
{
    int Kp_avrg_cnt = 0;
    int Kp_avrg = 0;
    int cnt = 0;
    int osc_cnt = 0;
    int period_time_ms = 0;
    int initial_time_ms = 0;
    char old_sign = 0;
    unsigned char rising_edge_counter = 0;
    unsigned int period_time_cnt = 0;
    double position_command = 0;
    arm_pid_instance_f32 position_pid;
    mobileAvrgStruct mAvrgStruct;
    memset((mobileAvrgStruct*) &mAvrgStruct, 0, sizeof(mobileAvrgStruct));
    const double pwm_ratio = (PWM_RATIO_COEFF_A * 7400.00 + PWM_RATIO_COEFF_B);
    const double error_max = 1.00; //distance error
    const int pwm_move_offset = 200;
    const int max_dist = 1000;
    position_pid.Kp = 16;
    position_pid.Ki = 0;
    position_pid.Kd = 200;

    encodersInit();
    motorsInit();
    telemetersInit();

    encodersReset();

    pidControllerInit(&position_pid);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(0, 0, "     TELEMETERS PID CAL", &Font_3x6);
        ssd1306DrawStringAtLine(0, 1, "  RIGHT TO START, LEFT TO EXIT", &Font_3x6);

//        ssd1306PrintfAtLine(0, 2, &Font_5x8, "Kp = %d", (uint32_t)zhonxCalib_data->pid_gyro.Kp);
//        ssd1306PrintfAtLine(0, 3, &Font_5x8, "Ki = %d", (uint32_t)(zhonxCalib_data->pid_gyro.Ki));
//        ssd1306PrintfAtLine(0, 4, &Font_5x8, "Kd = %d.10^-3", (uint32_t)(zhonxCalib_data->pid_gyro.Kd * 1000));
        ssd1306Refresh();

        if (expanderJoyFiltered() == JOY_LEFT)
        {
            return;
        }
        HAL_Delay(100);
    }
    ssd1306ClearScreen(MAIN_AREA);
    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
    ssd1306Refresh();
    HAL_Delay(1000);
    motorsDriverSleep(OFF);
    telemetersStart();

    while(expanderJoyFiltered() != JOY_LEFT)        //for manual testing
    {
        position_command = (pidController(&position_pid, (double) getTelemeterDist(TELEMETER_DL)
                                          - (double) getTelemeterDist(TELEMETER_DR)));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio));
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio));
        HAL_Delay(1);
    }
}
