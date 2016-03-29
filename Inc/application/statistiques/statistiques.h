/*
 * statistiques.h
 *
 *  Created on: 15 avr. 2015
 *      Author: Colin
 */

#ifndef STATISTIQUES_H_
#define STATISTIQUES_H_
/* Dependences */

#include "config/errors.h"
#include "config/module_id.h"

/* Error codes */

#define STATISTIQUES_MODULE_ID_E_SUCCESS  0
#define STATISTIQUES_MODULE_ID_ERROR    MAKE_ERROR(MAIN_CONTROL_MODULE_ID, 1)

#define MAVRG_NBITS 4 //todo optimize for several functions
#define MAVRG_BUFF_SIZE (1 << MAVRG_NBITS)

typedef struct
{
    unsigned int hist_series[MAVRG_BUFF_SIZE];
    unsigned int old_index;
    int hist_sum;
} mobileAvrgStruct;

/* Functions definitions */

int cStandardError(int statistical_series[], int statistical_serial_length);
float cVariance(int statistical_series[], int statistical_serial_length);
int cAverage(int statistical_series[], int statistical_serial_length);
int mobileAvrgInt(mobileAvrgStruct *mavrg, int new_sample);
int cStandardDeviation(int statistical_series[], int statistical_serial_length);
void eliminateExtremeValues(int *statistical_series, int *statistical_serial_length);
#endif /* STATISTIQUES_H_ */
