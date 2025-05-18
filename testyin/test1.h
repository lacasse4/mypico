

#ifndef TEST1_H
#define TEST1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

// vector
#include "./vector/flux_vector.h"
#include "./vector/flux_vectorOp.h"
#include "./vector/flux_complex.h"

// util
#include "./util/flux_util.h"
#include "./util/flux_wave.h"

// dsp
#include "./dsp/flux_window.h"
#include "./dsp/fft_algorithm.h"
// #include "./dsp/conv_algorithm.h"
// #include "./dsp/xcorr_algorithm.h"


// .
#include "../pitch/flux_base.h"

#ifdef __cplusplus
}
#endif

#endif