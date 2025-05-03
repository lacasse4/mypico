/**
 * @name fdetect
 * @details
 * Frequency Detection: this module implements functions to perform frequency detection
 * using a zero crossing algorithm. It detects falling edges on a GPIO pin, and measures the
 * period elapsed between two subsequent falling edges. The period is converted to a frequency.
 * Finally, the frequency is filtered:
 *  - large variations are sorted out by a non-linear filter (limit.h)
 *  - valid frequency values are smoothed by a single tap IIR filter (alpha.h)
 * 
 * Technology:
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 'input_pin' for frequency estimation
 *  - uses an interrupt handler on 'input_pin'
 *  - uses a repeating timer interrupt 
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-02
 */

 #include "pico/critical_section.h"

#include "fdetect.h"
#include "fedge.h"
#include "alpha.h"
#include "limit.h"

static volatile bool timer_elapsed = false;
static volatile bool data_valid = false;
static volatile double frequency;
static alpha_t *alpha;
static limit_t *limit;

static struct repeating_timer timer;

void flush();

// repeating timer callback
static bool timer_callback(__unused struct repeating_timer *t) 
{
    double raw_frequency; 

    frequency = 0.0;
    if (is_fedge_handler_done()) {
        raw_frequency = get_fedge_frequency();
        if (limit_next(limit, raw_frequency)) { 
            frequency = alpha_filter(alpha, raw_frequency);
            data_valid = true;
        }
        else {
            alpha_reset(alpha);
        }
    }

    timer_elapsed = true;

    // start next detection round of falling edges on INPUT_PIN
    arm_fedge_handler();

    return true;
}

// module initialisation. should be called first
void init_fdetect(uint input_pin, int32_t sleep_ms, double weight, double max_pct) 
{
    flush();

    // Setup falling edge interrupt routine on input_pin
    init_fedge_handler(input_pin); 

    // setup a repeating timer that starts at each sleep_ms period
    add_repeating_timer_ms(-sleep_ms, timer_callback, NULL, &timer);

    // create an IIR filter object
    alpha = create_alpha(weight);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(max_pct, 0.0);
    assert(limit);
}

// release resources previously allocated by init_fdetect()
void release_fdetect() 
{
    cancel_repeating_timer(&timer);
    release_alpha(alpha);
    release_limit(limit);
}

// check if the timer has elapsed
bool is_ready() 
{
    return timer_elapsed;
}

// get the estimated frequency. The frequency value is valid if true is returned.
bool get_frequency(double *frequency_out) 
{
    bool data_valid_out;

    data_valid_out = data_valid;
    *frequency_out = frequency;
    flush();

    return data_valid_out;
}

void flush() 
{
    timer_elapsed = false;
    data_valid = false;
    frequency = 0.0;
}