#ifndef _LIMIT_H
#define _LIMIT_H

#define LIMIT_STABLE      0
#define LIMIT_RESET       1

typedef struct limit limit_t;

limit_t* create_limit(double pct, double target);
void     release_limit(limit_t *limit);
int      limit_check(limit_t *limit, double value);
int      limit_next(limit_t *limit, double value);

#endif