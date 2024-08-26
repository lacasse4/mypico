/**
 * myblink.c
 * 
 * diagnostic program that flashes the board led and 
 * writes strings to the USB port
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"

int main() {

    int count = 0;

    stdio_init_all();
    printf("myblink\n");

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    while (true) {
        printf("%d\n", count++);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
        gpio_put(LED_PIN, 0);
        sleep_ms(250);
    }
}

