#include "pico/critical_section.h"

#include "rtimer.h"
#include "fedge.h"
#include "alpha.h"
#include "limit.h"

static volatile bool timer_elapsed = false;
static volatile bool data_valid = false;
static volatile double global_frequency;
static alpha_t *alpha;
static limit_t *limit;

static critical_section_t crit_sec;
static struct repeating_timer timer;

static bool rtimer_callback(__unused struct repeating_timer *t) 
{
    double raw_frequency; 
    double filtered_frequency = 0.0;
    bool local_data_valid;

    critical_section_enter_blocking(&crit_sec);
    data_valid = false;
    timer_elapsed = false;
    global_frequency = 0.0;
    critical_section_exit(&crit_sec);

    local_data_valid = is_fedge_handler_done();
    if (local_data_valid) {
        raw_frequency = get_fedge_frequency();
        local_data_valid = limit_next(limit, raw_frequency);
        if (local_data_valid) { 
            filtered_frequency = alpha_filter(alpha, raw_frequency);
        }
        else {
            alpha_reset(alpha);
        }
    }

    critical_section_enter_blocking(&crit_sec);
    data_valid = local_data_valid;
    timer_elapsed = true;
    if (data_valid) global_frequency = filtered_frequency;
    critical_section_exit(&crit_sec);

    // start next detection round of falling edges on INPUT_PIN
    arm_fedge_handler();

    // toggle Pico on board LED
    // toggle_led();           

    return true;
}

void init_rtimer(uint input_pin, int length, int32_t sleep_ms, double weight, double max_pct) 
{
    timer_elapsed = false;
    data_valid = false;

    // Setup falling edge interrupt routine on input_pin
    init_fedge_handler(input_pin, length); 

    critical_section_init(&crit_sec);
    add_repeating_timer_ms(-sleep_ms, rtimer_callback, NULL, &timer);

    // create an IIR filter object
    alpha = create_alpha(weight);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(max_pct, 0.0);
    assert(limit);
}

void release_rtimer() {
    critical_section_deinit(&crit_sec);
    cancel_repeating_timer(&timer);
    release_alpha(alpha);
    release_limit(limit);
}

bool is_rtimer_complete() 
{
    bool local_timer_elasped;

    critical_section_enter_blocking(&crit_sec);
    local_timer_elasped = timer_elapsed;
    critical_section_exit(&crit_sec);

    return local_timer_elasped;
}

bool get_rtimer_frequency(double *frequency) 
{
    bool local_data_valid;

    critical_section_enter_blocking(&crit_sec);
    local_data_valid = data_valid;
    *frequency = global_frequency;
    data_valid = false;
    timer_elapsed = false;
    global_frequency = 0.0;
    critical_section_exit(&crit_sec);

    return local_data_valid;
}
