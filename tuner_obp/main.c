/**
 * @name tuner_obp
 * @details
 * Simple bass/guitar tuner using the OBP algoritm 
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-20
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "pico/stdlib.h"

/*
 * Small utilities to show an "alive" signal and for debugging
 */

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

/*
 * Entry point for core0 processing
 *
 */

// GPIO pin on which data acquisition is done
#define INPUT_PIN       4

#define TIMER_US        100
#define SAMPLING_FREQ   (1000000/TIMER_US)

#define LOW_FREQ        35    // Hz
#define HIGH_FREQ       500   // Hz
#define LOW_PERIOD      (SAMPLING_FREQ/HIGH_FREQ-1)
#define HIGH_PERIOD     (SAMPLING_FREQ/LOW_FREQ+1)

#define SIGNAL_LENGTH   1024    // SIGNAL_LENGTH should be a power of 2 greater than HIGH_INDEX*2
#if SIGNAL_LENGTH < HIGH_INDEX*2
    #error SIGNAL_LENGTH must be greater than HIGH_INDEX*2
#endif
#define BIT_PER_WORD    32
#define BIT_ARRAY_SIZE  (SIGNAL_LENGTH/BIT_PER_WORD)	


static uint32_t bit_array[BIT_ARRAY_SIZE];
static uint32_t acorr_array[SIGNAL_LENGTH];
static int word_count = 0;
static int bit_count = 0;
static volatile bool done = false;
repeating_timer_t timer;


bool repeating_timer_callback(__unused struct repeating_timer *t) 
{
    bool pin_status;
    uint32_t word;
    // printf("%d\n", word_count);

    if (done) return true;

    pin_status = gpio_get(INPUT_PIN);
    if (pin_status) {
        word = bit_array[word_count];
        word |= 1 << bit_count;
        bit_array[word_count] = word;
    }

    bit_count++;
    if (bit_count == BIT_PER_WORD) {
        bit_count = 0;
        word_count++;
        if (word_count == BIT_ARRAY_SIZE) {
            done = true;
            word_count = 0;
            bit_count = 0;
        }
    }

    return true;
}

void print_bit_array(uint32_t *bit_array)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		for (int j = 0; j < BIT_PER_WORD; j++) {
			int bit = (bit_array[i] & (1 << j)) ? 1 : 0;
			printf("%d", bit);		
		}
	}
    printf("\n");
}

void shift_bit_array(uint32_t *bit_array)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		bit_array[i] = (bit_array[i] >> 1) | (bit_array[i + 1] << (BIT_PER_WORD - 1));
	}
}

void xor_bit_arrays(uint32_t *bit_array1_in, uint32_t *bit_array2_in, uint32_t *bit_array1_out)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		bit_array1_out[i] = bit_array1_in[i] ^ bit_array2_in[i];
	}
}

uint32_t sum_bit_array(uint32_t *bit_array, int start_bit, int end_bit)
{
	uint32_t sum = 0;
	static const uint8_t bit_count_table[256] = {
		0,1,1,2,1,2,2,3,1,2,2,3,2,3,3,4,1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
		2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
		1,2,2,3,2,3,3,4,2,3,3,4,3,4,4,5,2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,
		2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
		2,3,3,4,3,4,4,5,3,4,4,5,4,5,5,6,3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,
		3,4,4,5,4,5,5,6,4,5,5,6,5,6,6,7,4,5,5,6,5,6,6,7,5,6,6,7,6,7,7,8
	};

	if (start_bit < 0) start_bit = 0;
	if (end_bit > SIGNAL_LENGTH) end_bit = SIGNAL_LENGTH;
	if (start_bit >= end_bit) return 0;

	int start_word = start_bit / BIT_PER_WORD;
	int end_word = (end_bit - 1) / BIT_PER_WORD;

	for (int i = start_word; i <= end_word; i++) {
		uint32_t v = bit_array[i];
		int word_start = (i == start_word) ? (start_bit % BIT_PER_WORD) : 0;
		int word_end = (i == end_word) ? ((end_bit - 1) % BIT_PER_WORD) : (BIT_PER_WORD - 1);

		uint32_t mask;
		if (word_end == BIT_PER_WORD - 1 && word_start == 0) {
			mask = 0xFFFFFFFF;
		} else {
			mask = ((1U << (word_end - word_start + 1)) - 1) << word_start;
		}
		v &= mask;

		// Count only the bits in the masked region
		// Inline bit counting for up to 4 bytes (since uint32_t is 32 bits)
		sum += bit_count_table[v & 0xFF];
		sum += bit_count_table[(v >> 8) & 0xFF];
		sum += bit_count_table[(v >> 16) & 0xFF];
		sum += bit_count_table[(v >> 24) & 0xFF];
	}
	return sum;
}

void auto_diff_corr(uint32_t *bit_array, uint32_t *acorr_array, int length)
{
    uint32_t shifted_bit_array[BIT_ARRAY_SIZE];
    uint32_t result_bit_array[BIT_ARRAY_SIZE];

    memcpy(shift_bit_array, bit_array, sizeof(shift_bit_array));

    for (int i = 0; i < length; i++) {
        xor_bit_arrays(bit_array, shifted_bit_array, result_bit_array);
        acorr_array[i] = sum_bit_array(result_bit_array, i, i+HIGH_PERIOD);
        shift_bit_array(shifted_bit_array);
    }
}

int find_min_index(uint32_t *array, int length, int start_index)
{
    int min = UINT32_MAX;
    int imin = -1;

    for (int i = start_index; i < length; i++) {
        if (min > array[i]) {
            min = array[i];
            imin = i;
        }
    }
    return imin;
}


int main() {
    // double frequency;
    // int status;
    // uint32_t start_time = 0;
    // uint32_t elapsed_time, now;
    // uint32_t timestamp = 0;

    // initialize printing to console
    stdio_init_all();
    sleep_ms(100);
    printf("tuner_obp\n");

    // Inititialize on board LED (live signal)
    init_led();   
    
    // set input_pin
    gpio_set_input_enabled (INPUT_PIN, true);
    gpio_pull_up (INPUT_PIN);

    // set repeating timer to acquire bit stream
    memset(bit_array, 0, sizeof(bit_array));

    // set and start repeating timer
    add_repeating_timer_us(-TIMER_US, repeating_timer_callback, NULL, &timer);

    while (true) {
        while(!done); // wait for acquisition to complete

        cancel_repeating_timer(&timer);

        auto_diff_corr(bit_array, acorr_array, HIGH_PERIOD);
        int index = find_min_index(acorr_array, HIGH_PERIOD, LOW_PERIOD);
        printf("i: %d\n", index);

        // to be removed
        // now = time_us_32();
        // elapsed_time =  now - start_time;
        // start_time = now;
    }

}
