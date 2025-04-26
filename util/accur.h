#ifndef _ACCUR_H
#define _ACCUR_H

#define ACCUR_OK     0
#define ACCUR_LOW   -1
#define ACCUR_FAULT -2

typedef struct accur accur_t;

accur_t* create_accur(int n, double target);
void     release_accur(accur_t *accur);
void     accur_flush(accur_t *accur);
void     accur_set_target(accur_t *accur, double target);
int      accur_add(accur_t *accur, double value);
int      accur_results(accur_t *accur, double *accuracy, double *precision);

#endif