#include <stdlib.h>
#include "limit.h"

struct limit {
    int status;
    double pct;
    double target;
    double previous;
};

// create an limit_t object
limit_t* create_limit(double pct, double target)
{
    limit_t *limit = (limit_t *) malloc(sizeof(limit_t));
    if (!limit) return NULL;
    limit->status = LIMIT_RESET;
    limit->pct = pct;
    limit->target = target;
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
    return limit->target >= low && limit->target <= high;
}

// check if value is within limits, 
// if not, switch target and reset limit_t
int limit_next(limit_t *limit, double value)
{
    if (!limit) return 0;
    int status = limit_check(limit, value);
    if (!status) {
        limit->target = value;
    }
    return status;
}
