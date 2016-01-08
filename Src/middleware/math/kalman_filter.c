/*
 * kalman_filter.c
 *
 *  Created on: 6 janv. 2016
 *      Author: Colin
 */


void Kalman_filter_init (kalman_filter_params *params, float p, float q, float r)
{
	params->p = p;
	params->q = q;
	params->r = r; // Bruit du capteur.
}
float Kalman_filter (kalman_filter_params *params, float value_to_filtre)
{
	params->pc = params->p + params->q;
	params->k = params->pc / (params->pc + params->r);
	params->p = (1 - params->k) * params->pc;
	params->xp = params->xe;
	params->zp = params->xp;
	params->xe = params->k * (value_to_filtre - params->zp) + params->xp;
	return params->xe;
}
void Kalman_filter_reset_to_value (kalman_filter_params *params, float value)
{
	for (int i = 0; i < 10; ++i)
	{
		Kalman_filter(params, value);
	}
}
