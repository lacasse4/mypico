/**
 * @name tuner_count_no_wait
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This is an improvement over tuner_count program:
 *  - the falling edge counter status is also checked in order to improve the measurement speed
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

#include "accur.h"
#include "fdetect.h"

// GPIO pin on which zero crossing are detected
#define INPUT_PIN       14

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA           0.1

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
#define SLEEP_TIME_MS   200  // must be physically greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif

// Accuracy target for accuracy and precision measurments (accur.h)
#define ACCURACY_TARGET 213.07  // 555 astable test circuit

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


// Program entry point
int main() {
    int alive = 0;
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
    init_fdetect(INPUT_PIN, SLEEP_TIME_MS, ALPHA, MAX_FREQ_PCT);

    while (true) {
        
        if (is_ready()) {

            printf("  %5d", alive++);
            toggle_led();

            if (get_frequency(&frequency)) {
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
    release_fdetect();
}
