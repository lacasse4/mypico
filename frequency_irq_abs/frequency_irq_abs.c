/**
 * frequency_irq_abs.c
 *
 * Measure frequency on GPIO PIN_IRQ.
 * Detects absence of frequency 
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
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define LEN 10

#define MAX_FREQ    400         // in Hz
#define MIN_FREQ    75          // in Hz
#define TIMER_FREQ  1000000     // in Hz
#define MAX_PERIOD_US  (LEN*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LEN*TIMER_FREQ/MAX_FREQ)

#define SLEEP_TIME_MS 200  // must be physicaly greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be greater than MAX_PERIOD_US
#endif

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint PIN_IRQ = 14;
static bool led_on = true;

static volatile bool first = false;
static volatile bool done  = true;
static uint32_t start;
static uint32_t period;
static int counter;

void gpio_callback(uint gpio, uint32_t events) 
{
    uint32_t value = time_us_32();

    if (first) {
        start = value;
        first = false;
        return;
    }

    if (!done) {
        counter++;
        if (counter == LEN) {
            period = value - start;
            done = true;
        }
    }
}

void start_search_safe() 
{
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
    done  = false;
    first = true;
    period = 0;
    counter = 0;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);
}

void toggle_led() 
{
    if (led_on) {
        gpio_put(LED_PIN, 0);
    }
    else {
        gpio_put(LED_PIN, 1);
    }
    led_on = !led_on;
}

int main() {
    double frequency;

    stdio_init_all();
    printf("Frequency_irq_abs\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_set_input_enabled (PIN_IRQ, true);
    gpio_pull_up (PIN_IRQ);
    gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Wait forever
    while (true) {
        start_search_safe();

        toggle_led();

        // wait some time to compute period
        // the wait time must be longer than the time for detecting the lower frequency 
        sleep_ms(SLEEP_TIME_MS);

        if (done) {
            frequency = LEN * 1000000.0 / period;
            if (frequency > MAX_FREQ) {
                printf("Too high \n");
            }
            else if (frequency < MIN_FREQ) {
                printf("Too low \n");
            }
            else {
                printf(" %8.3f  \n", frequency);
            }
        }
        else {
            if (first) {
                printf("No signal \n");
            }
            else {
                printf("Too low \n");
            }
        }
    }
}