#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "alpha.h"

struct alpha {
    int status;
    double weight;
    double one_minus_weight;
    double tap;
};


// create an alpha_t object
alpha_t* create_alpha(double weight)
{
    alpha_t *alpha = (alpha_t *) malloc(sizeof(alpha_t));
    if (!alpha) return NULL;
    alpha->status = ALPHA_RESET;
    alpha->weight = weight;
    alpha->one_minus_weight = 1.0 - weight;
    return alpha;
}

// release an alpha_t object
void release_alpha(alpha_t *alpha) 
{
    free(alpha);
}

// set filter to to specified value
void alpha_reset(alpha_t *alpha) 
{
    if (!alpha) return;
    alpha->status = ALPHA_RESET;
}

// filter this value
double alpha_filter(alpha_t *alpha, double value)
{
    if (!alpha) return 0.0;
    if (alpha->status == ALPHA_RESET) {
        alpha->status = ALPHA_STABLE;
        alpha->tap = value;
    }
    else {
        alpha->tap = alpha->tap * alpha->one_minus_weight + value * alpha->weight;
    }
    return alpha->tap;
}
