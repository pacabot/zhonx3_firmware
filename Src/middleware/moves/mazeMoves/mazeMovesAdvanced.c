/**************************************************************************/
/*!
 @file    mazeMovesAdvanced
 @author  PLF (PACABOT)
 @date
 @version  0.0
 */
/**************************************************************************/
/* General declarations */
#include <stm32f4xx_hal.h>
#include <arm_math.h>
#include <config/basetypes.h>
#include <stm32f4xx_hal.h>
#include <string.h>
#include <stdint.h>

/* Application declarations */
#include <application/statistiques/statistiques.h>

/* Middleware declarations */
#include <middleware/controls/pidController/pidController.h>
#include <middleware/moves/mazeMoves/mazeMovesAdvanced.h>
#include <middleware/settings/settings.h>
#include <middleware/moves/basicMoves/basicMoves.h>
#include <middleware/controls/mainControl/mainControl.h>
#include <middleware/controls/mainControl/positionControl.h>

/* Peripheral declarations */
#include <peripherals/display/smallfonts.h>
#include <peripherals/display/ssd1306.h>
#include <peripherals/encoders/ie512.h>
#include <peripherals/expander/pcf8574.h>
#include <peripherals/gyroscope/adxrs620.h>
#include <peripherals/motors/motors.h>
#include <peripherals/telemeters/telemeters.h>

/* Declarations for this module */
#include <middleware/moves/mazeMoves/mazeMovesAdvanced.h>

int mazeMoveAdvancedTest(void)
{
    positionControlSetPositionType(GYRO);
    mainControlSetFollowType(NO_FOLLOW);
    basicMoveClothoid(500, 200, 200, 1000);
    while (hasMoveEnded() == TRUE);
    motorsBrake();
    HAL_Delay(1000);
    motorsDriverSleep(ON);

    return MAZE_MOVES_ADVANCED_E_SUCCESS;
}

//int mazeMoveAdvancedTest(void)
//{
//    double error = 0;
//    int Kp_avrg_cnt = 0;
//    int Kp_avrg = 0;
//    int cnt = 0;
//    int osc_cnt = 0;
//    int period_time_ms = 0;
//    int initial_time_ms = 0;
//    char old_sign = 0;
//    unsigned char rising_edge_counter = 0;
//    unsigned int period_time_cnt = 0;
//    double position_command = 0;
//    arm_pid_instance_f32 position_pid;
//    mobileAvrgStruct mAvrgStruct;
//    memset((mobileAvrgStruct*) &mAvrgStruct, 0, sizeof(mobileAvrgStruct));
//    const double pwm_ratio = (PWM_RATIO_COEFF_A * 8000.00 + PWM_RATIO_COEFF_B);
//    const double error_max = 0.1; //angular error
//    const int pwm_move_offset = 200;
//    const int max_dist = 1000;
//    position_pid.Kp = 5;
//    position_pid.Ki = 0;
//    position_pid.Kd = 300;
//    arm_pid_instance_f32 coefs;
//    int rv;
//
//    motorsInit();
//    adxrs620Init();
//    encodersInit();
//    telemetersInit();
//
//    encodersReset();
//    gyroResetAngle();
//
//    pidControllerInit(&position_pid);
//
//    while (expanderJoyFiltered() != JOY_RIGHT)
//    {
//        ssd1306ClearScreen(MAIN_AREA);
//        ssd1306DrawStringAtLine(0, 0, "       MANUAL DIAG TEST", &Font_3x6);
//        ssd1306DrawStringAtLine(0, 1, "  RIGHT TO START, LEFT TO EXIT", &Font_3x6);
//        ssd1306Refresh();
//
//        if (expanderJoyFiltered() == JOY_LEFT)
//        {
//            return MAZE_MOVES_ADVANCED_E_SUCCESS;
//        }
//        HAL_Delay(100);
//    }
//    ssd1306ClearScreen(MAIN_AREA);
//    ssd1306DrawStringAtLine(50, 1, "Wait", &Font_3x6);
//    ssd1306Refresh();
//    HAL_Delay(1000);
//    motorsDriverSleep(OFF);
//    telemetersStart();
//
//    while(expanderJoyFiltered() != JOY_LEFT)        //for manual testing
//    {
//        if (getTelemeterDist(TELEMETER_FL) < 200.00)
//            error = -1.00 * (200 - getTelemeterDist(TELEMETER_FL));
//        if (getTelemeterDist(TELEMETER_FR) < 200.00)
//            error = (200 - getTelemeterDist(TELEMETER_FR));
//
//        position_command = (pidController(&position_pid, error));
//
//        motorSet_DF(MOTOR_R, (int)(position_command * pwm_ratio) + 50);
//        motorSet_DF(MOTOR_L, (int)(-position_command * pwm_ratio) + 50);
//        HAL_Delay(1);
//    }
//    telemetersStop();
//    return MAZE_MOVES_ADVANCED_E_SUCCESS;
//}
