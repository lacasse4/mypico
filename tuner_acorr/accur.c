#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "accur.h"

#define MAX_N 30

struct accur {
    int count;
    int start;
    int n;
    double target;
    double values[MAX_N];
};

static double calculate_sigma(double *values, int count);
static double calculate_accuracy(double *values, int count, double target);

// create an accur_t object
accur_t* create_accur(int n, double target) 
{
    if (n > MAX_N || n <= 1) return NULL;
    accur_t *accur = (accur_t *) malloc(sizeof(accur_t));
    if (!accur) return NULL;
    accur->n = n;
    accur->target = target;
    accur_flush(accur);
    return accur;
}

// release an accur_t object
void release_accur(accur_t *accur) 
{
    free(accur);
}

// flush (set to 0) an accur_t object
void accur_flush(accur_t *accur) 
{
    if (!accur) return;
    accur->count = 0;
    accur->start = 0;
    for (int i = 0; i < MAX_N; i++) accur->values[i] = 0.0;
}

void accur_set_target(accur_t *accur, double target) 
{
    if (!accur) return;
    accur->target = target;
}

// add a new value to the sample's list
int accur_add(accur_t *accur, double new_value)
{
    if (!accur) return ACCUR_FAULT;
    // printf("  start: %d\n", accur->start);
    // printf("  count: %d\n", accur->count);
    // printf("  n: %d\n", accur->n);
    // printf("  xxx %d\n", (accur->start + accur->count) % accur->n);
    // printf("\n");

    // Add the new value to the circular buffer
    accur->values[(accur->start + accur->count) % accur->n] = new_value;
    if (accur->count < accur->n) {
        accur->count++;
    } else {
        accur->start = (accur->start + 1) % accur->n;
    }

    return accur->count >= accur->n ? ACCUR_OK : ACCUR_LOW;
}

int accur_results(accur_t *accur, double *accuracy, double *precision)
{
    if (!accur) return ACCUR_FAULT;

    if (accur->count >= accur->n) {
        *precision = calculate_sigma(accur->values + accur->start, accur->n);
        *accuracy = calculate_accuracy(accur->values + accur->start, accur->n, accur->target);
        return ACCUR_OK;
    }
    else {
        *accuracy = 0.0;
        *precision = 0.0;
        return ACCUR_LOW;
    }
}

// Function to calculate the standard deviation
double calculate_sigma(double *values, int count) {
    if (count == 0) return 0.0;

    double sum = 0.0, mean = 0.0, variance = 0.0;

    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    mean = sum / count;

    for (int i = 0; i < count; i++) {
        variance += (values[i] - mean) * (values[i] - mean);
    }
    variance /= count;

    return sqrt(variance);
}

// Function to calculate the accuracy
double calculate_accuracy(double *values, int count, double target) {
    if (count == 0) return 0.0;

    double sum = 0.0;
    for (int i = 0; i < count; i++) {
        sum += values[i];
    }
    double mean = sum / count;

    return fabs(mean - target);
}
