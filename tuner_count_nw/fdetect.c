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

#include <stdio.h>

#include "fdetect.h"
#include "fedge.h"
#include "alpha.h"
#include "limit.h"

static volatile bool timer_elapsed = false;
static int32_t sleep_time_ms;
static alarm_id_t alarm_id;
static alpha_t *alpha;
static limit_t *limit;

void flush();
void arm_detection();


int64_t alarm_callback(alarm_id_t id, void *user_data) 
{
    timer_elapsed = true;
    return 0;
}

// module initialisation. should be called first
void init_fdetect(uint input_pin, int32_t sleep_ms_in, double weight, double max_pct) 
{
    // keep sleep_ms (used in arm_detection())
    sleep_time_ms = sleep_ms_in;

    // Setup falling edge interrupt routine on input_pin
    init_fedge_handler(input_pin); 

    // start detection for the first time
    arm_detection();

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
    release_alpha(alpha);
    release_limit(limit);
}

// check if the timer has elapsed
bool is_ready() 
{
    // bool x = is_fedge_handler_done();
    // if (x || timer_elapsed) {
    //     // printf("ir: %d ", (x << 1) + timer_elapsed);
    //     printf("ir: %d ", (x << 1));
    // }
    // return x || timer_elapsed;
    return timer_elapsed || is_fedge_handler_done();
}

// get the estimated frequency. The frequency value is valid if true is returned.
bool get_frequency(double *frequency) 
{
    double raw_frequency; 
    bool data_valid;

    *frequency = 0.0;
    data_valid = false;

    // cancel the timer alarm
    cancel_alarm(alarm_id);

    // if the falling edge handler is done, a frequency is available
    if (is_fedge_handler_done()) {
        raw_frequency = get_fedge_frequency();
        if (limit_next(limit, raw_frequency)) { 
            *frequency = alpha_filter(alpha, raw_frequency);
            data_valid = true;
        }
        else {
            alpha_reset(alpha);
        }
    }

    // start next detection round of falling edges on INPUT_PIN
    arm_detection();
    return data_valid;
}

void arm_detection() 
{
    arm_fedge_handler();
    timer_elapsed = false;
    alarm_id = add_alarm_in_ms(sleep_time_ms, alarm_callback, NULL, false);
}
