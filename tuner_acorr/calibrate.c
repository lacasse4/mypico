#include <stdlib.h>
#include "calibrate.h"

struct calibrate {
    double slope;
    double intercept;
};

// create an calibrate_t object
calibrate_t* create_calibrate(int n, double *x, double *y)
{
    calibrate_t *calibrate = (calibrate_t *) malloc(sizeof(calibrate_t));
    if (!calibrate) return NULL;

    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < n; i++) {
        sumX += x[i];
        sumY += y[i];
        sumXY += x[i] * y[i];
        sumX2 += x[i] * x[i];
    }
    calibrate->slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    calibrate->intercept = (sumY - (calibrate->slope) * sumX) / n;

    return calibrate;
}

// release an calibrate_t object
void release_calibrate(calibrate_t *calibrate) 
{
    free(calibrate);
}


double calibrate_getY(calibrate_t *calibrate, double x) 
{
    if (!calibrate) return 0.0;
    return calibrate->slope * x + calibrate->intercept;
}
