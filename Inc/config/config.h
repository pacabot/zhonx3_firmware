/* config.h */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/**************************************************************************************/
/***************                 STM32 definitions                 ********************/
/**************************************************************************************/
#define ADC_VOLTAGE				3300   //STM32 ADC peripheral reference is 3300mV
#define ADC_COEFF 				ADC_VOLTAGE/4095 //ADC value/mV

/**************************************************************************************/
/***************                 Times definitions                 ********************/
/**************************************************************************************/
#define LOW_TIME_FREQ			100
#define HI_TIME_FREQ			10000
#define REGULAR_TIME_FREQ		50000
#define INJECTED_TIME_FREQ		5000
#define MULTIMMETER_TIME_FREQ	0.1

/**************************************************************************************/
/***************                 Gyro definitions                  ********************/
/**************************************************************************************/
#define GYRO_VRATIO				3300.00	//Gyro is running at 3300mV
#define GYRO_ROUT				90.00  	//90KHz Rout
#define GYRO_SENSITIVITY		6.00   	//Our example gyro is 6mV/deg/sec @5V
#define ROTATION_THRESHOLD		1.00   	//Minimum deg/sec to keep track of - helps with gyro drifting
#define GYRO_TIME_FREQ			INJECTED_TIME_FREQ //Gyro is clocking at 10KHz

#define GYRO_ZERO_VOLTAGE		(GYRO_VRATIO/2.00) 	//Gyro is zeroed at Vrate/2 (mV)
#define GYRO_OUTPUT_RATIO	    (GYRO_ROUT/(GYRO_ROUT+180.00)) 	//output resistor ratio (low-pass filter)
#define GYRO_OUT_SENSITIVITY	(GYRO_OUTPUT_RATIO*(GYRO_VRATIO/5000.00)*GYRO_SENSITIVITY) 	//1,32mV/deg/sec (3.3v R 60K)
#define GYRO_COEFF		        (GYRO_VRATIO/(4095.00*GYRO_OUT_SENSITIVITY*GYRO_TIME_FREQ)) //integration multiplier coeff

#define GYRO_T_SENSITIVITY 		9.00   //The temperature coefficient is ~9 mV/°C at 25°C
#define GYRO_T_OUT_SENSITIVITY	(GYRO_T_SENSITIVITY*(GYRO_VRATIO/5000.00)) // ~3,564 mV/°C at 25°C (3.3v)
#define GYRO_T_COEFF_A			(GYRO_VRATIO/(4095.00*GYRO_T_OUT_SENSITIVITY))
#define GYRO_T_COEFF_B			(-GYRO_VRATIO/(2.00*GYRO_T_OUT_SENSITIVITY)+25.00)

/**************************************************************************************/
/***************                 Temperature STM32                 ********************/
/**************************************************************************************/
#define STM32_VREFINT			3300.00	//1210.00 if use Vrefint
#define STM32_T_SENSITIVITY		2.50	//The temperature coefficient is ~2.5 mV/°C at 25°C
#define STM32_T_V25				760.00	//Voltage at 25 °C
#define STM32_T_COEFF_A			(STM32_VREFINT/(4095.00*STM32_T_SENSITIVITY))
#define STM32_T_COEFF_B			((-STM32_T_V25/STM32_T_SENSITIVITY)+25)

/**************************************************************************************/
/***************                 	VBAT                           ********************/
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

#endif // __CONFIG_H__

