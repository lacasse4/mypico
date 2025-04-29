#ifndef _FEDGE_H
#define _FEDGE_H

#include "pico/stdlib.h"

// falling_edge_handler() states 
#define FEH_IDLE     0
#define FEH_LAUNCH   1
#define FEH_COUNTING 2
#define FEH_DONE     3

void init_fedge_handler(uint input_pin, int length);
void arm_fedge_handler();
bool is_fedge_handler_done();
double get_fedge_frequency();

#endif