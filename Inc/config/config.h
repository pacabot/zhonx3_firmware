/* config.h */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <math.h>

/**************************************************************************************/
/***************                 STM32 definitions                 ********************/
/**************************************************************************************/
#define ADC_VOLTAGE				(3300)                //STM32 ADC peripheral reference is 3300mV
#define ADC_COEFF 				((ADC_VOLTAGE) / (4095))  //ADC value/mV

/**************************************************************************************/
/***************                 Times definitions                 ********************/
/**************************************************************************************/
#define HI_TIME_FREQ			    (20000.00) 	            //use for master inner loop
#define LOW_TIME_FREQ               (100.00)                //low inner loop
#define CONTROL_TIME_FREQ           (1000.00)               //use for control inner loop
#define LINE_FOLLOWER_TIME_FREQ     (1000.00)               //use for line follower inner loop
#define TELEMETERS_TIME_FREQ        ((2000.00) * (10.00))   //each telemeter use 1/10 of TELEMETERS_TIME_FREQ
#define GYRO_TIME_FREQ			    (2000.00)               //for adc injected freq
#define LINESENSORS_TIME_FREQ	    (4000.00)               //line sensor measure frequency
#define MULTIMMETER_TIME_FREQ       (0.10)                  //multimeter inner loop
#define MOTORS_FREQ				    (23000.00)	            //motor pwm freq

/**************************************************************************************/
/***************                 Gyro definitions                  ********************/
/**************************************************************************************/
#define GYRO_VRATIO				(3300.00)	//Gyro is running at 3300mV
#define GYRO_ROUT				(90.90)  	//90.9Kohm Rout
#define GYRO_SENSITIVITY		(6.00)   	//Our example gyro is 6mV/deg/sec @5V
#define ROTATION_THRESHOLD		(3.00)   	//Minimum deg/sec to keep track of - helps with gyro drifting

#define GYRO_ZERO_VOLTAGE		((GYRO_VRATIO) / (2.00)) 	//Gyro is zeroed at Vrate/2 (mV)
#ifdef GYRO_ROUT
#define GYRO_OUTPUT_RATIO	    ((GYRO_ROUT) / ((GYRO_ROUT) + (180.00))) 	//output resistor ratio (low-pass filter)
#else
#define GYRO_OUTPUT_RATIO	    (1) 	//output resistor ratio (low-pass filter)
#endif
#define GYRO_OUT_SENSITIVITY	((GYRO_OUTPUT_RATIO) * ((GYRO_VRATIO) / (5000.00)) * (GYRO_SENSITIVITY)) 	//1,32mV/deg/sec (3.3v Req 60K) saturation : 1363°/s
#define GYRO_A_COEFF		    ((GYRO_VRATIO) / ((4095.00) * (GYRO_OUT_SENSITIVITY) * (GYRO_TIME_FREQ))) //integration multiplier coeff

#define GYRO_T_SENSITIVITY 		(9.00)   //The temperature coefficient is ~9 mV/�C at 25�C
#define GYRO_T_OUT_SENSITIVITY	((GYRO_T_SENSITIVITY) * ((GYRO_VRATIO) / (5000.00))) // ~3,564 mV/�C at 25°C (3.3v)
#define GYRO_T_COEFF_A			((GYRO_VRATIO) / ((4095.00) * (GYRO_T_OUT_SENSITIVITY)))
#define GYRO_T_COEFF_B			((25.00) - ((GYRO_VRATIO) / ((2.00) * (GYRO_T_OUT_SENSITIVITY))))

//#define GYRO_B_COEFF			(1237.5938788750565 / GYRO_TIME_FREQ)

/**************************************************************************************/
/***************                 Temperature STM32                 ********************/
/**************************************************************************************/
#define STM32_VREFINT			(3300.00)	//1210.00 if use Vrefint
#define STM32_T_SENSITIVITY		(2.50)	//The temperature coefficient is ~2.5 mV/�C at 25�C
#define STM32_T_V25				(760.00)	//Voltage at 25 °C
#define STM32_T_COEFF_A			((STM32_VREFINT) / ((4095.00) * (STM32_T_SENSITIVITY)))
#define STM32_T_COEFF_B			((25.00) - ((STM32_T_V25) / (STM32_T_SENSITIVITY)))

/**************************************************************************************/
/***************                    	VBAT                       ********************/
/**************************************************************************************/
#define VBAT_R1_BRIDGE			(20.00)	//High bridge resistor (Kohms)
#define VBAT_R2_BRIDGE			(10.00)	//low bridge resistor(Kohms)

#define VBAT_BRIDGE_COEFF		(((STM32_VREFINT) / (4095.00)) * (((VBAT_R1_BRIDGE) + (VBAT_R2_BRIDGE)) / (VBAT_R2_BRIDGE)))

/**************************************************************************************/
/***************                     Telemeters                    ********************/
/**************************************************************************************/
#define DISTANCE_WALL_DIAG          (170.00)
#define DISTANCE_WALL_FRONT         (195.00)

/**************************************************************************************/
/***************                       Battery                     ********************/
/**************************************************************************************/
#define BATTERY_CELL_NUMBER					(2.00)	//2S
#define BATTERY_LOWER_VOLTAGE_NO_LOAD		((3000) * (BATTERY_CELL_NUMBER))	//https://learn.sparkfun.com/tutorials/battery-technologies/lithium-polymer
#define BATTERY_UPPER_VOLTAGE_NO_LOAD		((3700) * (BATTERY_CELL_NUMBER))
#define BATTERY_LOWER_VOLTAGE_OFFSET		((-)0.10) * (BATTERY_CELL_NUMBER))	//-0.1V/A
#define BATTERY_COEFF_A						(((BATTERY_UPPER_VOLTAGE_NO_LOAD) - (BATTERY_LOWER_VOLTAGE_NO_LOAD)) / (100.00))
#define BATTERY_COEFF_B						((BATTERY_LOWER_VOLTAGE_NO_LOAD) / (BATTERY_COEFF_A))

/**************************************************************************************/
/***************                 Mechanical Constants              ********************/
/**************************************************************************************/
#define WHEEL_DIAMETER			(24.30) 	//Wheel diameter in millimeters
#define WHEELS_DISTANCE			(63.20)	//Distance between right and left wheels
#define WHEELS_SPACING			(25.96)	//Distance between front and rear wheels
#define	GEAR_RATIO				(50.00 / 15.00)	//wheel gear teeth per motor gear teeth
#define ENCODER_RESOLUTION  	(2047.00)	//Number steps per revolution (IE512)

#define STEPS_PER_WHEEL_REV		((ENCODER_RESOLUTION) * (GEAR_RATIO))	//Number steps per wheel revolution
#define MM_PER_WHEEL_REV		((M_PI) * (WHEEL_DIAMETER))		//Number of millimeters per wheel revolution
#define STEPS_PER_MM			((STEPS_PER_WHEEL_REV) / (MM_PER_WHEEL_REV))	//Number of steps per millimeter

/**************************************************************************************/
/***************                  Robot Dimensions                 ********************/
/******** you can see also Inc/application/solverMaze.h for more properties ***********/
/**************************************************************************************/
#define Z3_WIDTH				(72.50)
#define Z3_LENGHT				(98.40)
#define Z3_HEIGHT				(23.70)
#define Z3_CENTER_BACK_DIST     (34.00)
#define Z3_CENTER_FRONT_DIST    ((Z3_LENGHT) - (Z3_CENTER_BACK_DIST))

/**************************************************************************************/
/***************                   Maze Properties                 ********************/
/******** you can see also Inc/application/solverMaze.h for more properties ***********/
/**************************************************************************************/
#define WALL_THICKNESS			(12.00)
#define HALF_WALL_THICKNESS		((WALL_THICKNESS) / (2.00))
#define CELL_LENGTH				(179.00)
#define HALF_CELL_LENGTH		((CELL_LENGTH) / (2.00))
#define MAZE_SIZE				(17)

/**************************************************************************************/
/***************                  Moves Constants                  ********************/
/**************************************************************************************/
#define OFFSET_DIST                 (15.00)
#define MAIN_DIST                   ((CELL_LENGTH) - ((OFFSET_DIST) * (2.00)))

#define DEADZONE_VIEWING_OFFSET     (99.00)
#define DEADZONE_DIST               (CELL_LENGTH)   //Distance between the start of the cell and doubt area
#define DEADZONE                    (100.00)         //doubt area
#define DEADZONE_CHECKWALL_DIST     (5.00)

#define WALL_FOLLOW_DIAG_DIST       (83.50)     //85 DARK, 83 MEDDLE
#define WALL_FOLLOW_MAX_DIAG_DIST   (120.00)

/**************************************************************************************/
/***************                     Maze speed                    ********************/
/**************************************************************************************/

#define RUN1_SPEED_ROTATION         (500)
#define RUN1_MIN_SPEED_TRANSLATION  (500)
#define RUN1_MAX_SPEED_TRANSLATION  (1500)

#define RUN2_SPEED_ROTATION         (600)
#define RUN2_MIN_SPEED_TRANSLATION  (600)
#define RUN2_MAX_SPEED_TRANSLATION  (3000)

#define SCAN_SPEED_ROTATION         (400)
#define SCAN_MIN_SPEED_TRANSLATION  (400)
#define SCAN_MAX_SPEED_TRANSLATION  (800)

#define SAFE_SPEED_ROTATION         (400)
#define SAFE_SPEED_TRANSLATION      (400)

#define RETURN_START_CELL

/**************************************************************************************/
/***************                 Physical Constants                ********************/
/**************************************************************************************/
#define MAX_SPEED               (4500.00)   //mm/s
#define MAX_ACCEL               (7000.00)   //mm/s/s

#define MAX_CURVE_ACCEL         (1300.00)   //mm/s
#define MAX_CURVE_SPEED         (800.00)   //mm/s

#define MAX_ROTATE_ACCEL        (500.00)   //°/s/s perhaps wrong scale //todo verify correct scale
#define MAX_TURN_ACCEL          (4000.00)   //°/s/s perhaps wrong scale //todo verify correct scale

/**************************************************************************************/
/***************                 Motors Constants                  ********************/
/**************************************************************************************/

#define PWM_RATIO_COEFF_A		((-0.50) / (6000.00))	//compute pwm ratio for limit motor voltage
#define PWM_RATIO_COEFF_B		(1.50)			    //PWM_RATIO_COEFF_A * battery voltage + PWM_RATIO_COEFF_B = TRUE MOTOR PWM

#define MOTORS_PERIOD			(1000)  //PWM scale 1000 = 100%

/*Calculations For Motor*/
#define MOTORS_INDUCTANCE       (0.000065)    //H
#define MOTORS_RESISTANCE       (4.30)
#define MOTORS_PWM_FREQUENCY    (((((MOTORS_RESISTANCE) / (MOTORS_INDUCTANCE)) / ((2.00) * (PI))) * 2)

/**************************************************************************************/
/***************                  Flash Constants                  ********************/
/**************************************************************************************/
/* Base address of the Flash sectors */
// Sectors reserved for Zhonx Program
#define ADDR_FLASH_SECTOR_0     (0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     (0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     (0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     (0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     (0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     (0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     (0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     (0x08060000) /* Base @ of Sector 7, 128 Kbytes */
// Sectors reserved for Zhonx Data (512KB)
#define ADDR_FLASH_SECTOR_8     (0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     (0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    (0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    (0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

#define CONFIG_FLASH_SECTOR_BUFFER_SIZE	(16 * 1024)
#define CONFIG_FLASH_NB_FLASH_DEVICES	(1)

// Stored mazes
#define STORED_MAZES_ADDR                (ADDR_FLASH_SECTOR_8)
// Calibration data
#define CALIBRATION_DATA_ADDR            (ADDR_FLASH_SECTOR_9)
// Telemeters profiles
#define TELEMETERS_PROFILES_ADDR         (ADDR_FLASH_SECTOR_10)
// Address in flash for ZHONX informations
#define CONFIG_ZHONX_INFO_ADDR           (ADDR_FLASH_SECTOR_11)

/**************************************************************************************/
/***************                 EEPROM Constants                  ********************/
/**************************************************************************************/
#define CONFIG_EEPROM_SIZE              (8192)
#define CONFIG_EEPROM_PAGE_SIZE         (32)
#define CONFIG_EEPROM_MAX_PAGE_COUNT    ((CONFIG_EEPROM_SIZE) / (CONFIG_EEPROM_PAGE_SIZE))

/**************************************************************************************/
/***************                     bluetooth                     ********************/
/**************************************************************************************/
//#define DISABLE_BLUETOOTH

/**************************************************************************************/
/***************                   Misc Constants                  ********************/
/**************************************************************************************/
// Define this variable to enable Command Line mode
// Note: If not defined, Hexadecimal Command mode is used by default
#define CONFIG_USE_CMDLINE
/**************************************************************************************/
/***************               sleep and kill params               ********************/
/**************************************************************************************/
#define KILL_WEN_UNUSED

#endif // __CONFIG_H__

