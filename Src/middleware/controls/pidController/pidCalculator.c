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
    const double error_max = 0.12; //angular error
    const int pwm_move_offset = 200;
    const int max_dist = 1000;
    position_pid.Kp = 100;
    position_pid.Ki = 0;
    position_pid.Kd = 0;
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
        ssd1306DrawStringAtLine(0, 0, "         POSITION PID CAL",              &Font_3x6);
        ssd1306DrawStringAtLine(0, 1, "  RIGHT TO START, LEFT TO EXIT",    &Font_3x6);

        ssd1306PrintfAtLine(0, 2, &Font_5x8, "Kp = %d", (uint32_t)coefs.Kp);
        ssd1306PrintfAtLine(0, 3, &Font_5x8, "Ki = %d.10^-3", (uint32_t)(coefs.Ki * 1000));
        ssd1306PrintfAtLine(0, 4, &Font_5x8, "Kd = %d", (uint32_t)(coefs.Kd * 1000));
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
    //start 1s for stabilise
    for (cnt = 0; cnt < 1000; cnt++)
    {
        position_command = (pidController(&position_pid, gyroGetAngle()));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio));
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio));
        HAL_Delay(1);
    }
    //measure loop
    while ((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00 < max_dist)
    {
        cnt++;
        position_command = (pidController(&position_pid, gyroGetAngle()));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio) + pwm_move_offset);
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio) + pwm_move_offset);

        if (cnt > 5)
        {
            if ((gyroGetAngle() < error_max) && (gyroGetAngle() > -error_max))
            {
                position_pid.Kp += 1;   //increase Kp coeff
            }
            else
            {
                position_pid.Kp -= 1;   //decrease Kp coeff
            }
            Kp_avrg += position_pid.Kp;
            Kp_avrg_cnt++;
            if (position_pid.Kp < 100)
                position_pid.Kp = 100;
            pidControllerInit(&position_pid);
            cnt = 0;
        }

        //compute oscillation time and decrease Kp for search critical point
        if (gyroGetAngle() > error_max)
        {
            if (old_sign != POSITIVE)
            {
                old_sign = POSITIVE;
                rising_edge_counter++;
                if (rising_edge_counter >= 2)
                {
                    osc_cnt++;
                    period_time_ms += (HAL_GetTick() - initial_time_ms);
                    rising_edge_counter = 0;
                }
                else
                {
                    initial_time_ms = HAL_GetTick();
                }
            }
        }
        if (gyroGetAngle() < -error_max)
        {
            if (old_sign != NEGATIVE)
            {
                old_sign = NEGATIVE;
            }
        }
        HAL_Delay(1);
    }
    motorsBrake();
    period_time_ms /= osc_cnt;
    Kp_avrg /= Kp_avrg_cnt;

    coefs.Kp = ((double)Kp_avrg * 0.60);
    coefs.Ki = 2.00 * coefs.Kp / (period_time_ms / 1000.00);
    coefs.Kd = coefs.Kp * (period_time_ms / 1000.00) / 8.00;

    // save PID coefficients in Flash memory
    rv = flash_write(zhonxSettings.h_flash,
                     (unsigned char *)&zhonxCalib_data->pid_gyro,
                     (unsigned char *)&coefs, sizeof(arm_pid_instance_f32));
    if (rv != FLASH_DRIVER_E_SUCCESS)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306PrintfAtLine(0, 1, &Font_5x8, "FAILED To write Gyro PID values");
        ssd1306Refresh();
        HAL_Delay(2000);
    }

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(0, 0, &Font_5x8, "Kcr= %d Tcr= %d ms",  (uint32_t)Kp_avrg, (uint32_t)period_time_ms);
    ssd1306PrintfAtLine(0, 1, &Font_5x8, "Kp = %d", (uint32_t)coefs.Kp);
    ssd1306PrintfAtLine(0, 2, &Font_5x8, "Ki = %d.10^-3", (uint32_t)(coefs.Ki * 1000));
    ssd1306PrintfAtLine(0, 3, &Font_5x8, "Kd = %d", (uint32_t)(coefs.Kd * 1000));
    ssd1306Refresh();
    HAL_Delay(2000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
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
    const double pwm_ratio = (PWM_RATIO_COEFF_A * 8000.00 + PWM_RATIO_COEFF_B);
    const double error_max = 1.00; //distance error
    const int pwm_move_offset = 200;
    const int max_dist = 1000;
    position_pid.Kp = 1;
    position_pid.Ki = 0;
    position_pid.Kd = 0;

    encodersInit();
    motorsInit();
    telemetersInit();

    encodersReset();

    pidControllerInit(&position_pid);

    while (expanderJoyFiltered() != JOY_RIGHT)
    {
        ssd1306ClearScreen(MAIN_AREA);
        ssd1306DrawStringAtLine(35, 0, "POSITION PID CAL", &Font_3x6);
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
    //start 1s for stabilise
    for (cnt = 0; cnt < 1000; cnt++)
    {
        position_command = (pidController(&position_pid, (double) getTelemeterDist(TELEMETER_DL)
                                          - (double) getTelemeterDist(TELEMETER_DR)));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio));
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio));
        HAL_Delay(1);
    }
    //measure loop
    while ((encoderGetDist(ENCODER_L) + encoderGetDist(ENCODER_R)) / 2.00 < max_dist)
    {
        cnt++;
        position_command = (pidController(&position_pid, (double) getTelemeterDist(TELEMETER_DL)
                                          - (double) getTelemeterDist(TELEMETER_DR)));

        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio) + pwm_move_offset);
        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio) + pwm_move_offset);

        if (cnt > 5)
        {
            if ((((double) getTelemeterDist(TELEMETER_DL) - (double) getTelemeterDist(TELEMETER_DR)) < error_max) &&
                    (((double) getTelemeterDist(TELEMETER_DL) - (double) getTelemeterDist(TELEMETER_DR) > -error_max)))
            {
                position_pid.Kp += 1;   //increase Kp coeff
            }
            else
            {
                position_pid.Kp -= 1;   //decrease Kp coeff
            }
            Kp_avrg += position_pid.Kp;
            Kp_avrg_cnt++;
            if (position_pid.Kp < 1)
                position_pid.Kp = 1;
            pidControllerInit(&position_pid);
            cnt = 0;
        }

        //compute oscillation time and decrease Kp for search critical point
        if (((double) getTelemeterDist(TELEMETER_DL) - (double) getTelemeterDist(TELEMETER_DR)) > error_max)
        {
            if (old_sign != POSITIVE)
            {
                old_sign = POSITIVE;
                rising_edge_counter++;
                if (rising_edge_counter >= 2)
                {
                    osc_cnt++;
                    period_time_ms += (HAL_GetTick() - initial_time_ms);
                    rising_edge_counter = 0;
                }
                else
                {
                    initial_time_ms = HAL_GetTick();
                }
            }
        }
        if (((double) getTelemeterDist(TELEMETER_DL) - (double) getTelemeterDist(TELEMETER_DR)) < -error_max)
        {
            if (old_sign != NEGATIVE)
            {
                old_sign = NEGATIVE;
            }
        }
        HAL_Delay(1);
    }
    motorsBrake();
    period_time_ms /= osc_cnt;
    Kp_avrg /= Kp_avrg_cnt;

    double Kp = ((double)Kp_avrg * 0.60);
    double Ki = 2.00 * Kp / ((double)period_time_ms / 1000.00);
    double Kd = Kp * ((double)period_time_ms / 1000.00) / 8.00;

    ssd1306ClearScreen(MAIN_AREA);
    ssd1306PrintfAtLine(0, 0, &Font_5x8, "Kcr= %d Tcr= %d ms",  (uint32_t)Kp_avrg, (uint32_t)period_time_ms);
    ssd1306PrintfAtLine(0, 1, &Font_5x8, "Kp = %d", (uint32_t)Kp);
    ssd1306PrintfAtLine(0, 2, &Font_5x8, "Ki = %d.10^-3", (uint32_t)(Ki));
    ssd1306PrintfAtLine(0, 3, &Font_5x8, "Kd = %d", (uint32_t)(Kd * 1000));
    ssd1306Refresh();
    telemetersStop();
    HAL_Delay(2000);
    motorsDriverSleep(ON);
    while (expanderJoyFiltered() != JOY_LEFT);
}

void pidCalculator(void)
{
    accelMotor_GetStepResponse();
}
