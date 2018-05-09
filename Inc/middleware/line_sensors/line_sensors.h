/*
 * @file line_sensors.h
 * @Author Colin Roubaud
 * @date 8 may 2018
 * @version 0.1
 */

#ifndef INC_MIDDLEWARE_LINE_SENSORS_LINE_SENSORS_H_
#define INC_MIDDLEWARE_LINE_SENSORS_LINE_SENSORS_H_

/**
 * @def MAX_FLOOR_SENSORS_VAL
 * @brief define the max value of the calibrate range
 */
#define MAX_FLOOR_SENSORS_VAL 1000

/**
 * @def LINE_SENSOR_CNT
 * @brief define the number of
 */
#define FLOOR_SENSOR_CNT 5

/**
 * @def CALIB_ANGL
 * @brief define the angle of the calibration for the floor sensors
 */
#define CALIB_ANGL 120

/**
 * @struct CalibrateFloorSensors
 * @brief this struct contain all the data of a floor sensor
 * this strict contain sensor by sensors the dynamic range of a sensor
 */
typedef struct
{
	double min[FLOOR_SENSOR_CNT]; /*!< the minimal ADC value of each sensors*/
	double max[FLOOR_SENSOR_CNT]; /*!< the maximal ADC value of each sensors*/
}FloorSensorsCalib;

struct FloorSensorsValues
{
	int val[FLOOR_SENSOR_CNT]; /*!< the value of all sensors in range [0, MAX_FLOOR_SENSORS_VAL]*/
};

/**
 * @fn FloorSensorsCalib floorSensorCalib(void);
 * @brief take the range for line sensors
 * this function will do a rotation over the line to take for all line sensors
 * the minimal and maximal value the calibration struct is store forget the line
 *
 * @return the CalibrateFloorSensors struct for this line and floor
 */
FloorSensorsCalib floorSensorCalib(void);

/**
 * @fn struct FloorSensorsValues getFloorSensorsValues(FloorSensorsCalib calib)
 * @brief read and return the floor sensors values
 * this funtion will read the ADC value of the floor sensors
 * and map them in range [0, MAX_FLOOR_SENSORS_VAL]
 *
 * @param calib the calibration set for this floor
 * @return the values of each sensors
 */
struct FloorSensorsValues getFloorSensorsValues(FloorSensorsCalib calib);

/**
 * @fn int getLinePos(FloorSensorsCalib calib)
 * @brief return the line pos
 *
 * @return the line pos [-3 MAX_FLOOR_SENSORS_VAL, 3 MAX_FLOOR_SENSORS_VAL;]
 */
int getLinePos(void);

/**
 * @fn void setFloorSensorsCalib(FloorSensorsCalib calib)
 * @brief set a calibration set
 * set a new set of calibration for read floor sensors
 *
 * @param calib the new calibration set
 */
void setFloorSensorsCalib(FloorSensorsCalib calib);

#endif /* INC_MIDDLEWARE_LINE_SENSORS_LINE_SENSORS_H_ */
