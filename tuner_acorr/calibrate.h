#ifndef _CALIBRATE_H
#define _CALIBRATE_H

typedef struct calibrate calibrate_t;

calibrate_t* create_calibrate(int n, double *x, double *y);
void   release_calibrate(calibrate_t *calibrate);
double calibrate_getY(calibrate_t *calibrate, double x);

#endif