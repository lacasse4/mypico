/**
 * @name fdetect
 * @details
 * Frequency Detection: this module implements functions to perform frequency detection
 * using a zero crossing algorithm. It detects falling edges on a GPIO pin, and measures the
 * period elapsed between two subsequent falling edges. The period is converted to a frequency.
 * Finally, the frequency is filtered:
 *  - large variations are sorted out by a non-linear filter (limit.h)
 *  - valid frequency values are smoothed by a single tap IIR filter (alpha.h)
 * An alarm timer limits the falling edge search to 'sleep_time_ms' milliseconds
 * 
 * Technology:
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 'input_pin' for frequency estimation
 *  - uses an interrupt handler on 'input_pin'
 *  - uses an alarm timer interrupt 
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-02
 */

#include "fdetect.h"
#include "fedge.h"
#include "alpha.h"
#include "limit.h"

static volatile bool timer_elapsed = false;
static int32_t sleep_time_ms;
static alarm_id_t alarm_id;
static alpha_t *alpha;
static limit_t *limit;

// prototypes
void set_timer();

// alarm timer call back 
int64_t alarm_callback(alarm_id_t id, void *user_data) 
{
    timer_elapsed = true;
    return 0;
}

// module initialisation. should be called first
void fdetect_init(uint input_pin, int length, int32_t sleep_time_ms_in, double weight, double max_pct) 
{
    // an alarm timer is as a timeout mechanism if there is not signal on input_pin 
    sleep_time_ms = sleep_time_ms_in;   // store the time out internally
    set_timer();

    // initialize edge detection
    fedge_init(input_pin, length); 

    // launch edge detetion
    fedge_launch();

    // create an IIR filter object
    alpha = create_alpha(weight);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(max_pct, 0.0);
    assert(limit);
}

// release resources previously allocated by init_fdetect()
void fdetect_release() 
{
    cancel_alarm(alarm_id);
    release_alpha(alpha);
    release_limit(limit);
}

// check if the timer has elapsed
bool fdetect_is_read_ok() 
{
    return fedge_is_read_ok();
}

// check if the timer has elapsed
bool fdetect_is_timer_elapsed() 
{
    return timer_elapsed;
}

bool fdetect_no_falling_edge() 
{
    return fedge_no_falling_edge();
}

// get the estimated frequency. The frequency value is valid if true is returned.
bool fdetect_get_frequency(double *frequency) 
{
    double raw_frequency; 
    bool data_valid;

    *frequency = 0.0;
    data_valid = false;

    // cancel the timer alarm
    cancel_alarm(alarm_id);

    // get frequency
    raw_frequency = fedge_get_frequency();
    if (limit_next(limit, raw_frequency)) { 
        *frequency = alpha_filter(alpha, raw_frequency);
        data_valid = true;
    }
    else {
        fedge_launch();
        limit_reset(limit);
        alpha_reset(alpha);
    }

    // set timer to react if there no signal on input_pin
    set_timer();
    return data_valid;
}

void fdetect_reset_search()
{
    cancel_alarm(alarm_id);
    fedge_launch();
    limit_reset(limit);
    alpha_reset(alpha);
    set_timer();
}

void set_timer() 
{
    timer_elapsed = false;
    alarm_id = add_alarm_in_ms(sleep_time_ms, alarm_callback, NULL, false);
}
