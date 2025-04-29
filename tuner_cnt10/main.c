/**
 * @name tuner_cnt10
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This program is an enhancement of tuner_count. 
 * The algorithm has LENGTH slot for time counting.
 * This improves the refresh rate (increased 10 fold).  
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2024-09-26
 */

/*
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110
 * D3   146.83
 * G3   196
 * B3   246.94
 * E4   329.63
 */

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/critical_section.h"

#include "accur.h"
#include "alpha.h"
#include "limit.h"

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA       0.1

// Accuracy target for accuracy and precision measurments (accur.c)
#define ACCURACY_TARGET 200

// A non-linear filter (limit.c) is used to discard output frequencies 
// that are to far from the previously measured frequency.
// MAX_FREQ_PCT is that maximum percentage allowed from the previous measurment.
#define MAX_FREQ_PCT    0.10

// number of falling edges counted to evaluate PIN_IRQ frequency
#define LENGTH 10  

// frequency detection parameters
#define MAX_FREQ    400         // in Hz
#define MIN_FREQ    75          // in Hz
#define TIMER_FREQ  1000000     // in Hz
#define MAX_PERIOD_US  (LENGTH*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LENGTH*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS 200  // must be physically greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif

// variables used to switch the Pico on board LED (alive signal)
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// GPIO pin on which zero crossing are detected
#define INPUT_PIN 14

// falling_edge_handler() states 
#define FEH_IDLE     0
#define FEH_LAUNCH   1
#define FEH_COUNTING 2
#define FEH_DONE     3

// variables used in falling_edge_handler()
static volatile int irq_state = FEH_IDLE;       // quiet at startup
static volatile uint32_t irq_start;
static volatile uint32_t irq_period;
static volatile int irq_counter;

// Interrupt handler on INPUT_PIN. Detects the period between two falling edges.
// To arm the counter, call arm_falling_edge_handler()
// To check the counter status, call is_falling_edge_handler_done()
void falling_edge_handler(uint gpio, uint32_t events) 
{
    // disable interrupts from INPUT_PIN during the execution of this routine
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, false);

    uint32_t time_stamp = time_us_32();

    // exit if in IDLE or DONE state
    if (irq_state == FEH_DONE && irq_state == FEH_IDLE) return;

    if (irq_state == FEH_COUNTING) {
        irq_counter++;
        if (irq_counter == LENGTH) {
            irq_period = time_stamp - irq_start;
            irq_state = FEH_DONE;
        }
    }

    if (irq_state == FEH_LAUNCH) {
        irq_start = time_stamp;
        irq_state = FEH_COUNTING;
    }

    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

// Setup the interrupt handler on PIN_IRQ
void init_falling_edge_handler() 
{
    gpio_set_input_enabled (INPUT_PIN, true);
    gpio_pull_up (INPUT_PIN);
    gpio_set_irq_enabled_with_callback(INPUT_PIN, GPIO_IRQ_EDGE_FALL, true, &falling_edge_handler);
}

// Arm the the counter safely
void arm_falling_edge_handler() 
{
    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, false);
    irq_state = FEH_LAUNCH;
    irq_period = 0;
    irq_counter = 0;
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, true);
}

// Check if we went though LENGTH falling edges
bool is_falling_edge_handler_done()
{
    int state;

    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, false);
    state = irq_state;
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, true);

    return state == FEH_DONE;
}

// Get the frequency from falling_edge_handler() data
// Note: check that is_falling_edge_handler_done() == FEH_DONE before performing this call.
double get_frequency_from_handler() 
{
    int state;
    uint32_t period;

    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, false);
    state = irq_state;
    period = irq_period;
    gpio_set_irq_enabled (INPUT_PIN, GPIO_IRQ_EDGE_FALL, true);

    if (state == FEH_DONE) return LENGTH * 1000000.0 / period;
    else return 0.0;
}


// Inititialize Pico on board LED
void init_led() 
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

// Toggle Pico on board LED
void toggle_led() 
{
    static int led_on = 0;
    gpio_put(LED_PIN, led_on);
    led_on = !led_on;
}


bool timer_elapsed = false;
bool data_valid = false;
double global_frequency;
alpha_t *alpha;
limit_t *limit;

critical_section_t crit_sec;
struct repeating_timer timer;

bool repeating_timer_callback(__unused struct repeating_timer *t) 
{
    double frequency; 
    double filtered_frequency = 0.0;
    bool local_data_valid;

    critical_section_enter_blocking(&crit_sec);
    data_valid = false;
    timer_elapsed = false;
    global_frequency = 0.0;
    critical_section_exit(&crit_sec);

    local_data_valid = is_falling_edge_handler_done();
    if (local_data_valid) {
        frequency = get_frequency_from_handler();
        local_data_valid = limit_next(limit, frequency);
        if (local_data_valid) { 
            filtered_frequency = alpha_filter(alpha, frequency);
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
    arm_falling_edge_handler();

    // toggle Pico on board LED
    toggle_led();           

    return true;
}

void init_repeating_timer() 
{
    timer_elapsed = false;
    data_valid = false;

    critical_section_init(&crit_sec);
    add_repeating_timer_ms(-SLEEP_TIME_MS, repeating_timer_callback, NULL, &timer);

    // create an IIR filter object
    alpha = create_alpha(ALPHA);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(MAX_FREQ_PCT, 0.0);
    assert(limit);
}

void release_repeating_timer() {
    critical_section_deinit(&crit_sec);
    cancel_repeating_timer(&timer);
    release_alpha(alpha);
    release_limit(limit);
}

bool is_timer_complete() 
{
    bool local_timer_elasped;

    critical_section_enter_blocking(&crit_sec);
    local_timer_elasped = timer_elapsed;
    critical_section_exit(&crit_sec);

    return local_timer_elasped;
}

bool get_filtered_frequency(double *frequency) 
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


// program entry point
int main() {
    int alive = 0;
    bool data_valid;
    int statistic_status;
    double frequency;
    double accuracy;
    double precision;
    accur_t *accur;


    stdio_init_all();

    // create an accuracy and precision measurment object 
    accur = create_accur(10, ACCURACY_TARGET);
    assert(accur);

    // create an IIR filter object
    alpha = create_alpha(ALPHA);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(MAX_FREQ_PCT, 0.0);
    assert(limit);

    // Inititialize on board LED (live signal)
    init_led();     

    // Setup interrupt routine on PIN_IRQ
    init_falling_edge_handler(); 

    // Set a repeating alarm that is called at each SLEEP_TIME_US
    init_repeating_timer();

    while (true) {
        
        if (is_timer_complete()) {

            printf("  %5d", alive++);
            data_valid = get_filtered_frequency(&frequency);
            if (data_valid) {
                printf("  filt: %6.2lf", frequency);
                statistic_status = accur_add(accur, frequency);
                if (statistic_status == ACCUR_OK) {
                    accur_results(accur, &accuracy, &precision);
                    printf("  acc: % -5.3lf", accuracy);
                    printf("  prc: %5.3lf", precision);
                }
            }
            else {
                accur_flush(accur);
            }
            printf("\n");
        }
    }

    release_accur(accur);
    release_repeating_timer();
}
