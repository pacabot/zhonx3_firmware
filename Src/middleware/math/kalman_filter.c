/*
 * kalman_filter.c
 *
 *  Created on: 6 janv. 2016
 *      Author: Colin
 */

#include "middleware/math/kalman_filter.h"
#include "stdbool.h"

void kalman_filter_init(kalman_filter_params *params, float p, float q, float r)
{
    params->p = p;
    params->q = q;
    params->r = r; // Bruit du capteur.
}
float kalman_filter(kalman_filter_params *params, float value_to_filtre)
{
    params->pc = params->p + params->q;
    params->k = params->pc / (params->pc + params->r);
    params->p = (1 - params->k) * params->pc;
    params->xp = params->xe;
    params->zp = params->xp;
    params->xe = params->k * (value_to_filtre - params->zp) + params->xp;
    return params->xe;
}
void kalman_filter_reset_to_value(kalman_filter_params *params, float value)
{
    for (int i = 0; i < 10; ++i)
    {
        kalman_filter(params, value);
    }
}
void kalman_filter_array(int array[], int array_length)
{
    kalman_filter_params filter_params;
    bool doAgain = true;
    bool sens = true;
    kalman_filter_init(&filter_params, 1.0, 10.0, 9.0);
    while (doAgain)
    {
        doAgain = false;
        if (sens)
        {
            kalman_filter_reset_to_value(&filter_params, array[0]);
            for (int i = 0; i < array_length; i++)
            {
                array[i] = kalman_filter(&filter_params, array[i]);
                if (i > 0 && (array[i - 1] - array[i]) < 0)
                {
                    doAgain = true;
                }
            }
        }
        else
        {
            kalman_filter_reset_to_value(&filter_params, array[array_length - 1]);
            for (int i = (array_length - 1); i >= 0; i--)
            {
                array[i] = kalman_filter(&filter_params, array[i]);
                if (i < 148 && (array[i] - array[i + 1]) < 0)
                {
                    doAgain = true;
                }
            }
        }
        sens = !sens;
    }
}
