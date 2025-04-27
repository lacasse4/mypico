/**
 * @name tuner_count
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (PIN_IRQ) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This program originated from frequency_irq_i2c. i2c logic was removed.  
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
#define ESTIMATION_LENGTH 10  

// frequency detection parameters
#define MAX_FREQ    400         // in Hz
#define MIN_FREQ    75          // in Hz
#define TIMER_FREQ  1000000     // in Hz
#define MAX_PERIOD_US  (ESTIMATION_LENGTH*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (ESTIMATION_LENGTH*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS 200  // must be physically greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif

// variables used to switch the Pico on board LED (alive signal)
const uint LED_PIN = PICO_DEFAULT_LED_PIN;

// definitions for the interrupt handler
#define PIN_IRQ 14
#define COUNTER_LAUNCH   1
#define COUNTER_COUNTING 2
#define COUNTER_DONE     3

static volatile int irq_state = COUNTER_DONE;   // irq_handler quiet at startup
static volatile uint32_t start;
static volatile uint32_t period;
static volatile int counter;

// Interrupt handler on PIN_IRQ
// It evaluates the time taken for PIN_IRQ to go through LEN falling egdes. 
// To arm the counter, call arm_counter_safe()
// To check the counter status, call is_counter_done()
void irq_counter(uint gpio, uint32_t events) 
{
    uint32_t time_stamp = time_us_32();

    if (irq_state == COUNTER_DONE) return;

    if (irq_state == COUNTER_COUNTING) {
        counter++;
        if (counter == ESTIMATION_LENGTH) {
            period = time_stamp - start;
            irq_state = COUNTER_DONE;
        }
    }

    if (irq_state == COUNTER_LAUNCH) {
        start = time_stamp;
        irq_state = COUNTER_COUNTING;
    }
}

// Arm the the counter safely
void arm_counter_safe() 
{
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
    irq_state = COUNTER_LAUNCH;
    period = 0;
    counter = 0;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);
}

int is_counter_done()
{
    int state;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
    state = irq_state;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);
    return state == COUNTER_DONE;
}

// Setup the interrupt handler on PIN_IRQ
void init_counter() 
{
    gpio_set_input_enabled (PIN_IRQ, true);
    gpio_pull_up (PIN_IRQ);
    gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &irq_counter);
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

// program entry point
int main() {
    int alive = 0;
    int statistic_status;
    double frequency;
    double filtered_frequency;
    double accuracy;
    double precision;

    accur_t *accur;
    alpha_t *alpha;
    limit_t *limit;

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
    init_counter();     

    while (true) {
        // Start a single evaluation of 'period'
        arm_counter_safe();

        // Toggle Pico on board LED
        toggle_led();           

        // Wait some time to compute input waveform period.
        // SLEEP_TIME must be greater than the time required to detect the lowest frequency (MAX_PERIOD)
        sleep_ms(SLEEP_TIME_MS);

        // Update displays if the the evaluation of 'period' is done
        printf("  %d", alive++);
        if (is_counter_done()) {
            frequency = ESTIMATION_LENGTH * 1000000.0 / period;
            printf("  freq: %6.2lf", frequency);
            if (limit_next(limit, frequency)) { 
                filtered_frequency = alpha_filter(alpha, frequency);
                printf("  filt: %6.2lf", filtered_frequency);
                statistic_status = accur_add(accur, filtered_frequency);
                if (statistic_status == ACCUR_OK) {
                    accur_results(accur, &accuracy, &precision);
                    printf("  acc: % -5.3lf", accuracy);
                    printf("  prc: %5.3lf", precision);
                }
            }
            else {
                alpha_reset(alpha);
                accur_flush(accur);
            }
        }
        printf("\n");
    }
}