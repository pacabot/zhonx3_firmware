/* config.h */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <math.h>

/**************************************************************************************/
/***************                 STM32 definitions                 ********************/
/**************************************************************************************/
#define ADC_VOLTAGE				3300   //STM32 ADC peripheral reference is 3300mV
#define ADC_COEFF 				ADC_VOLTAGE/4095 //ADC value/mV

/**************************************************************************************/
/***************                 Times definitions                 ********************/
/**************************************************************************************/
#define LOW_TIME_FREQ			100
#define HI_TIME_FREQ			1000.00 	//use for pids inner loop
#define TELEMETERS_TIME_FREQ	1600 * 10  		//each telemeter use 1/10 of TELEMETERS_TIME_FREQ
#define GYRO_TIME_FREQ			4000.00
#define LINESENSORS_TIME_FREQ	GYRO_TIME_FREQ 	//same timer
#define MULTIMMETER_TIME_FREQ	0.1f
#define MOTORS_FREQ				20000	//motor pwm freq

/**************************************************************************************/
/***************                 Gyro definitions                  ********************/
/**************************************************************************************/
#define GYRO_VRATIO				3300.00	//Gyro is running at 3300mV
#define GYRO_ROUT				90.90  	//90.9Kohm Rout
#define GYRO_SENSITIVITY		6.00   	//Our example gyro is 6mV/deg/sec @5V
#define ROTATION_THRESHOLD		3.00   	//Minimum deg/sec to keep track of - helps with gyro drifting

#define GYRO_ZERO_VOLTAGE		(GYRO_VRATIO/2.00) 	//Gyro is zeroed at Vrate/2 (mV)
#ifdef GYRO_ROUT
#define GYRO_OUTPUT_RATIO	    (GYRO_ROUT/(GYRO_ROUT+180.00)) 	//output resistor ratio (low-pass filter)
#else
#define GYRO_OUTPUT_RATIO	    1 	//output resistor ratio (low-pass filter)
#endif
#define GYRO_OUT_SENSITIVITY	(GYRO_OUTPUT_RATIO*(GYRO_VRATIO/5000.00)*GYRO_SENSITIVITY) 	//1,32mV/deg/sec (3.3v Req 60K)
#define GYRO_A_COEFF		    (GYRO_VRATIO/(4095.00*GYRO_OUT_SENSITIVITY*GYRO_TIME_FREQ)) //integration multiplier coeff

#define GYRO_T_SENSITIVITY 		9.00   //The temperature coefficient is ~9 mV/°C at 25°C
#define GYRO_T_OUT_SENSITIVITY	(GYRO_T_SENSITIVITY*(GYRO_VRATIO/5000.00)) // ~3,564 mV/°C at 25°C (3.3v)
#define GYRO_T_COEFF_A			(GYRO_VRATIO/(4095.00*GYRO_T_OUT_SENSITIVITY))
#define GYRO_T_COEFF_B			(-GYRO_VRATIO/(2.00*GYRO_T_OUT_SENSITIVITY)+25.00)

#define GYRO_B_COEFF			(1237.5938788750565 / GYRO_TIME_FREQ)
//#define GYRO_B_COEFF			1.2400995740599703// / GYRO_TIME_FREQ)//FORCE COEFF_B
#define GYRO_GAIN_COMPENSATION	0.95

/**************************************************************************************/
/***************                 Temperature STM32                 ********************/
/**************************************************************************************/
#define STM32_VREFINT			3300.00	//1210.00 if use Vrefint
#define STM32_T_SENSITIVITY		2.50	//The temperature coefficient is ~2.5 mV/°C at 25°C
#define STM32_T_V25				760.00	//Voltage at 25 °C
#define STM32_T_COEFF_A			(STM32_VREFINT/(4095.00*STM32_T_SENSITIVITY))
#define STM32_T_COEFF_B			((-STM32_T_V25/STM32_T_SENSITIVITY)+25)

/**************************************************************************************/
/***************                    	VBAT                       ********************/
/**************************************************************************************/
#define VBAT_R1_BRIDGE			20.00	//High bridge resistor (Kohms)
#define VBAT_R2_BRIDGE			10.00	//low bridge resistor(Kohms)

#define VBAT_BRIDGE_COEFF		((STM32_VREFINT/4095) * ((VBAT_R1_BRIDGE + VBAT_R2_BRIDGE) / VBAT_R2_BRIDGE))

/**************************************************************************************/
/***************                     Telemeters                    ********************/
/**************************************************************************************/
#define DEFAULT_LEFT_FRONT_OFFSET
#define DEFAULT_RIGHT_FRONT_OFFSET
#define DEFAULT_LEFT_DIAG_OFFSET
#define DEFAULT_RIGHT_DIAG_OFFSET

#define DEFAULT_HIGHTER_INTERVAL
#define DEFAULT_LOWER_INTERVAL

/**************************************************************************************/
/***************                       Battery                     ********************/
/**************************************************************************************/
#define BATTERY_CELL_NUMBER					2	//2S
#define BATTERY_LOWER_VOLTAGE_NO_LOAD		(3.6f * BATTERY_CELL_NUMBER)
#define BATTERY_UPPER_VOLTAGE_NO_LOAD		(4.2f * BATTERY_CELL_NUMBER)
#define BATTERY_LOWER_VOLTAGE_OFFSET		(-0.1f * BATTERY_CELL_NUMBER)	//-0.1V/A

/**************************************************************************************/
/***************                 Mechanical Constants              ********************/
/**************************************************************************************/
#define WHEEL_DIAMETER			23.90	//Wheel diameter in millimeters
#define WHEELS_DISTANCE			63.20	//Distance between right and left wheels
#define WHEELS_SPACING			25.96	//Distance between front and rear wheels
#define	GEAR_RATIO				(50.00/15.00)	//wheel gear teeth per motor gear teeth
#define ENCODER_RESOLUTION  	2047.00	//Number steps per revolution (IE512)

#define STEPS_PER_WHEEL_REV		(ENCODER_RESOLUTION*GEAR_RATIO)	//Number steps per wheel revolution
#define MM_PER_WHEEL_REV		((M_PI) * (WHEEL_DIAMETER))		//Number of millimeters per wheel revolution
#define STEPS_PER_MM			((STEPS_PER_WHEEL_REV) / (MM_PER_WHEEL_REV))	//Number of steps per millimeter

//#define DARK
#define MEDDLE

#if defined MEDDLE && defined DARK
#error you must define only one robot
#endif
#if (!defined MEDDLE) && (!defined DARK)
#error  you must define one robot
#endif

/**************************************************************************************/
/***************                  Robot Dimensions                 ********************/
/******** you can see also Inc/application/solverMaze.h for more properties ***********/
/**************************************************************************************/
#define Z3_CENTER_BACK_DIST	    34.00
#define Z3_WIDTH				72.50
#define Z3_LENGHT				98.40
#define Z3_HEIGHT				23.70

/**************************************************************************************/
/***************                   Maze Properties                 ********************/
/******** you can see also Inc/application/solverMaze.h for more properties ***********/
/**************************************************************************************/
#define WALL_THICKNESS			12.00
#define HALF_WALL_THICKNESS		WALL_THICKNESS / 2.00
#define CELL_LENGTH				179.20
#define HALF_CELL_LENGTH		CELL_LENGTH / 2.00
#define MAZE_SIZE				16

/**************************************************************************************/
/***************                 Physical Constants                ********************/
/**************************************************************************************/
#define MAX_SPEED				4000.0	//mm/s
#define MAX_ACCEL				4000.0	//mm/s²
//#define MAX_DECEL				8000.0	//mm/s²

#define MAX_TURN_SPEED			500.0	//mm/s
#define MAX_TURN_ACCEL			8000.0	//mm/s²

/**************************************************************************************/
/***************                 Motors Constants                  ********************/
/**************************************************************************************/

#define PWM_RATIO_COEFF_A		(-0.50/6000.00)	//compute pwm ratio for limit motor voltage
#define PWM_RATIO_COEFF_B		1.50				//PWM_RATIO_COEFF_A * battery voltage + PWM_RATIO_COEFF_B = TRUE MOTOR PWM

#define MOTORS_PERIOD			1000

#endif // __CONFIG_H__

