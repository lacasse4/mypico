/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

#define LEN 50

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint PIN_IRQ = 14;
static bool led_on = true;
static volatile bool first = false;
static volatile bool done  = true;
static uint32_t start;
static uint32_t period = 0;
static double frequency;
static double last_frequency = 0.0;

void gpio_callback(uint gpio, uint32_t events) 
{
    static int counter = 0;
    uint32_t value = time_us_32();

        if (led_on) {
            gpio_put(LED_PIN, 0);
        }
        else {
            gpio_put(LED_PIN, 1);
        }
        led_on = !led_on;

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
            counter = 0;
        }
    }
}

int main() {
    stdio_init_all();
    printf("Frequency_IRQ\n");

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);

    gpio_set_input_enabled (PIN_IRQ, true);
    gpio_pull_up (PIN_IRQ);
    gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);

    // Wait forever
    while (true) {
        gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
        done  = false;
        first = true;
        gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);

        while(!done);

        if (period > 0) {
            frequency = LEN * 1000000.0/ period;
            printf("%8.3f  %8.5f\n", frequency, fabs(frequency-last_frequency));
            last_frequency = frequency;
        }
        else {
            printf("XXXXXXXX\n");
        }

    }
}