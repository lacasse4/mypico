#ifndef _RTIMER_H
#define _RTIMER_H

#include "pico/stdlib.h"

void init_rtimer(uint input_pin, int length, int32_t sleep_ms, double weight, double max_pct);
void release_rtimer();
bool is_rtimer_complete();
bool get_rtimer_frequency(double *frequency);

#endif