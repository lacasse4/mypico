// Test program for bit array operations

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <string.h>
#include <math.h>

// #include "testdata.h"

#define SIGNAL_LENGTH 128
#define BIT_PER_WORD 32
#define START_BIT 20
#define END_BIT 108
#define BIT_ARRAY_SIZE (SIGNAL_LENGTH/BIT_PER_WORD)	

uint32_t bit_array[BIT_ARRAY_SIZE];
float signal_in[SIGNAL_LENGTH];

void generate_testdata(float *signal)
{
	for (int i = 0; i < SIGNAL_LENGTH; i++) {
		signal[i] = sinf(1 + 2 * M_PI * i * 10 / SIGNAL_LENGTH);
	}
}

void convert_signal_to_bit(float *signal, uint32_t *bit_array)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		bit_array[i] = 0;
		for (int j = 0; j < BIT_PER_WORD; j++) {
			bit_array[i] |= (signal[i * BIT_PER_WORD + j] > 0) ? (1 << j) : 0;
		}
	}
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

void print_array(float *signal, uint32_t *bit_array)
{
	int bit;

	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		for (int j = 0; j < BIT_PER_WORD; j++) {
			bit = (bit_array[i] & (1 << j)) ? 1 : 0;
			printf("%d %f\n", bit, signal[i * BIT_PER_WORD + j]);
		}
	}
}

void print_bit_array(uint32_t *bit_array)
{
	// ANSI escape code for red: \033[31m, reset: \033[0m
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		for (int j = 0; j < BIT_PER_WORD; j++) {
			int bit_index = i * BIT_PER_WORD + j;
			int bit = (bit_array[i] & (1 << j)) ? 1 : 0;
			if (bit_index >= START_BIT && bit_index < END_BIT) {
				printf("\033[31m%d\033[0m", bit); // print in red
			} else {
				printf("%d", bit);
			}
		}
	}
}

int main()
{
	uint32_t bit_array1[BIT_ARRAY_SIZE];
	uint32_t bit_array2[BIT_ARRAY_SIZE];
	uint32_t bit_array3[BIT_ARRAY_SIZE];
	uint32_t sum;

	printf("testobp\n");

	generate_testdata(signal_in);
	convert_signal_to_bit(signal_in, bit_array1);
	memcpy(bit_array2, bit_array1, sizeof(bit_array1));

	for (int i = 0; i < 20; i++) {
		xor_bit_arrays(bit_array1, bit_array2, bit_array3);
		sum = sum_bit_array(bit_array3, START_BIT, END_BIT);

		printf("i = %d\n", i);
		print_bit_array(bit_array1); 	printf("\n");
		print_bit_array(bit_array2);    printf("\n");
		print_bit_array(bit_array3);    printf(" %d\n\n", sum);

		shift_bit_array(bit_array2);
	}
	return 0;
}


