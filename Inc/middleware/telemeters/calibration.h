/*
 * calibration.h
 *
 *  Created on: 10 avr. 2015
 *      Author: Colin
 */

#ifndef CALIBRATION_H_
#define CALIBRATION_H_

#define CALIBRATION_E_SUCCESS  0
#define CALIBRATION_E_ERROR    MAKE_ERROR(CALIBRATION_MODULE_ID, 1)

#define NUMBER_OF_CASE 		50
#define DISTANCE_MEASURED	300
#define NUMBER_OF_MILLIMETER_PER_LOOP NUMBER_OF_CASE/DISTANCE_MEASURED
#define NUMBER_OF_MEASURE_BY_STEP 50
void telemeter_calibration();
#endif /* CALIBRATION_H_ */
