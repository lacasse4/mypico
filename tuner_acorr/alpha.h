#ifndef _ALPHA_H
#define _ALPHA_H

#define ALPHA_STABLE      0
#define ALPHA_RESET       1

typedef struct alpha alpha_t;

alpha_t* create_alpha(double weight);
void     release_alpha(alpha_t *alpha);
void     alpha_reset(alpha_t *alpha);
double   alpha_filter(alpha_t *alpha, double value);

#endif