/*
 * statistiques.c
 *
 *  Created on: 15 avr. 2015
 *      Author: Colin
 */

#include "application/statistiques/statistiques.h"
#include "math.h"

int cStandardError (int statistical_series[], int statistical_serial_length)
{
	return sqrtf(cVariance(statistical_series,statistical_serial_length));
}

float cVariance (int statistical_series[], int statistical_serial_length)
{
	//float variance=0;
	double variance=0;// TODO test if in double the precision is better
	int average =cAverage(statistical_series,statistical_serial_length);
	for (int i = 0; i<statistical_serial_length; i++)
	{
		variance+=powf((statistical_series[i]-average),2);
	}
	variance/=statistical_serial_length;
	return (float)variance;
}

int cStandardDeviation (int statistical_series[], int statistical_serial_length)
{
	return sqrtf(cVariance(statistical_series,statistical_serial_length));
}

int cAverage (int statistical_series[], int statistical_serial_length)
{
	long average=0;
	for (int i = 0 ; i < statistical_serial_length ; i++)
	{
		average+=(long)statistical_series[i];
	}
	return (int)(average/=statistical_serial_length);
}

void eliminateExtremeValues (int *statistical_series, int *statistical_serial_length)
{
	int average = cAverage (statistical_series, *statistical_serial_length);
	int standardError = cStandardError (statistical_series, *statistical_serial_length);
	for (int i=0;i<*statistical_serial_length;i++)
	{
		if(statistical_series[i] > average+standardError || statistical_series[i] < (average - standardError))
		{
			statistical_series[i]=-1;
		}
	}
	int i=0;
	for (int i2=0; i2<*statistical_serial_length;i2++)
	{
		if((i2+1)<*statistical_serial_length && statistical_series[i2]==-1)
		{
			i2++;
		}
		statistical_series[i]=statistical_series[i2];
		i++;
	}
	*statistical_serial_length=i;
}
