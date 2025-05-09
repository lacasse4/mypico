#ifndef _FEDGE_H
#define _FEDGE_H

#include "pico/stdlib.h"

// maximum number of period summed between falling edges to evaluate frequency
#define MAX_LENGTH      20

// fedge_handler() states 
#define FEH_IDLE     0
#define FEH_LOADING  1
#define FEH_COUNTING 2
#define FEH_READ_OK  3

void fedge_init(uint input_pin, int length);
void fedge_launch();
bool fedge_is_read_ok();
double fedge_get_frequency();

#endif