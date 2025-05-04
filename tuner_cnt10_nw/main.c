/**
 * @name tuner_cnt10_nw
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This is an improvement over tuner_count_no_wait program:
 * instead of waiting for LENGTH falling edges to occur (LENGTH is set as 50), 
 * a measurement is issued at each falling edges, but period measurment
 * is still done over LENGTH falling edges.  To do so, fedge.c
 * was adapted modified to include LENGTH measuring buckets.
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-03
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

#include "accur.h"
#include "fdetect.h"

// GPIO pin on which zero crossing are detected
#define INPUT_PIN       14

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA           0.01

// A non-linear filter (limit.c) is used to discard output frequencies 
// that are to far from the previously measured frequency.
// MAX_FREQ_PCT is that maximum percentage allowed from the previous measurment.
#define MAX_FREQ_PCT    0.10


// frequency detection parameters
#define MAX_FREQ        400         // in Hz
#define MIN_FREQ        75          // in Hz
#define TIMER_FREQ      1000000     // in Hz
#define MAX_PERIOD_US  (LENGTH*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LENGTH*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS   300   // must be physically greater than MAX_PERIOD_US
#if (SLEEP_TIME_MS*1000) <= MAX_PERIOD_US
    #error SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif


// Accuracy target for accuracy and precision measurments (accur.h)
#define ACCURACY_TARGET 214.60  // to fit a 555 astable test circuit

// Inititialize Pico on board LED
void init_led() 
{
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

// Toggle Pico on board LED
void toggle_led() 
{
    static int led_on = 0;
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    led_on = !led_on;
}

void print_invalid()
{
    static char c[4] = {'|', '/', '-', '\\'};
    static int i = 0;
    printf("     %c  no signal  %c       ", c[i], c[i]);
    i = (i +1) % 4;
}

// Program entry point
int main() {
    int statistic_status;
    double frequency;
    double accuracy;
    double precision;
    accur_t *accur;

    // initialize printing to console
    stdio_init_all();

    // create an accuracy and precision measurment object 
    accur = create_accur(10, ACCURACY_TARGET);
    assert(accur);

    // Inititialize on board LED (live signal)
    init_led();     

    // Set a repeating alarm that is called at each SLEEP_TIME_US
    sleep_ms(100);  // it seems that the internal timer takes some time to startup.
    fdetect_init(INPUT_PIN, SLEEP_TIME_MS, ALPHA, MAX_FREQ_PCT);

    while (true) {
        
        if (fdetect_is_ready()) {

            toggle_led();

            if (fdetect_get_frequency(&frequency)) {
                printf("  f: %6.2lf", frequency);
                statistic_status = accur_add(accur, frequency);
                if (statistic_status == ACCUR_OK) {
                    accur_results(accur, &accuracy, &precision);
                    // printf("  acc: % -5.3lf", accuracy);
                    printf("  p: %6.4lf", precision);
                }
            }
            else {
                print_invalid();
                accur_flush(accur);
            }
            printf("\r");
        }
    }

    release_accur(accur);
    fdetect_release();
}
