#include <stdlib.h>
#include "limit.h"

struct limit {
    double pct;
    double target;
    // double previous;
    int status;
};

// create an limit_t object
limit_t* create_limit(double pct, double target)
{
    limit_t *limit = (limit_t *) malloc(sizeof(limit_t));
    if (!limit) return NULL;
    limit->pct = pct;
    limit->target = target;
    limit->status = FIRST_TIME;
    return limit;
}

// release an limit_t object
void release_limit(limit_t *limit) 
{
    free(limit);
}

// check if value is within limits
int limit_check(limit_t *limit, double value)
{
    if (!limit) return 0;
    double gap  = value * limit->pct;
    double low  = value - gap;
    double high = value + gap;
    if (limit->target >= low && limit->target <= high) return LIMIT_WITHIN;
    return LIMIT_OUTSIDE;
}

// check if value is within limits, if not, set value as the new target
int limit_next(limit_t *limit, double value)
{
    if (!limit) return 0;
    if (limit->status == FIRST_TIME) {
        limit->status = !FIRST_TIME;
        limit->target = value;
        return LIMIT_WITHIN;
    } 
    int status = limit_check(limit, value);
    if (status == LIMIT_OUTSIDE) limit->target = value;
    return status;
}

void limit_reset(limit_t *limit)
{
    limit->status = FIRST_TIME;
}
