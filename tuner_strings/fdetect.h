#ifndef _FDETECT_H
#define _FDETECT_H

#include "pico/stdlib.h"

void fdetect_init(uint input_pin, int length, int32_t sleep_ms, double weight, double max_pct);
void fdetect_release();
bool fdetect_is_read_ok();
bool fdetect_is_timer_elapsed();
bool fdetect_no_falling_edge();
bool fdetect_get_frequency(double *frequency);
void fdetect_reset_search();

#endif