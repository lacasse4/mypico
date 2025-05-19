

#ifndef PITCH_H
#define PITCH_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>

// vector
#include "../pitch/vector/flux_vector.h"
#include "../pitch/vector/flux_vectorOp.h"
#include "../pitch/vector/flux_complex.h"

// util
#include "../pitch/util/flux_util.h"
#include "../pitch/util/flux_wave.h"

// dsp
#include "../pitch/dsp/flux_window.h"
#include "../pitch/dsp/fft_algorithm.h"
#include "../pitch/dsp/conv_algorithm.h"
#include "../pitch/dsp/xcorr_algorithm.h"

// base
#include "../pitch/flux_base.h"

#ifdef __cplusplus
}
#endif

#endif