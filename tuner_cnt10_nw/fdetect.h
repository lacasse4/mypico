#ifndef _FDETECT_H
#define _FDETECT_H

#include "pico/stdlib.h"

// number of period summed between falling edges to evaluate frequency
#define LENGTH      20

void fdetect_init(uint input_pin, int32_t sleep_ms, double weight, double max_pct);
void fdetect_release();
bool fdetect_is_ready();
bool fdetect_get_frequency(double *frequency);

#endif