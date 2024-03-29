/*
 * bezier_curves.c
 *
 *  Created on: 24 mars 2015
 *      Author: Colin
 */

#include <math.h>
#include "middleware/math/bezierCurves.h"

void courbe_de_bezier(int pointA[2], int pointB[2], int pointC[2], int pointD[2], point *tableOfPoint, int numberOfStep)
{
    int i;
    for (i = 0; i <= numberOfStep; ++i)
    {
        double t = (double) i / (double) numberOfStep;

        double a = pow((1.0 - t), 3.0);
        double b = 3.0 * t * pow((1.0 - t), 2.0);
        double c = 3.0 * pow(t, 2.0) * (1.0 - t);
        double d = pow(t, 3.0);

        double x = a * pointA[0] + b * pointB[0] + c * pointC[0] + d * pointD[0];
        double y = a * pointA[1] + b * pointB[1] + c * pointC[1] + d * pointD[1];
        tableOfPoint[i].x = (char)x;
        tableOfPoint[i].y = (char)y;
    }
}
