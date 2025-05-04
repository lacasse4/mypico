#ifndef _FDETECT_H
#define _FDETECT_H

#include "pico/stdlib.h"

void init_fdetect(uint input_pin, int32_t sleep_ms, double weight, double max_pct);
void release_fdetect();
bool is_ready();
bool get_frequency(double *frequency);

#endif