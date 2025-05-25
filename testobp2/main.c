#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>


#define SAMPLING_FREQ   10000 // Hz

#define LOW_FREQ        35    // Hz
#define HIGH_FREQ       600   // Hz
#define LOW_PERIOD      (SAMPLING_FREQ/HIGH_FREQ-1)
#define HIGH_PERIOD     (SAMPLING_FREQ/LOW_FREQ+1)

#define WINDOW_LENGTH   1024    // WINDOW_LENGTH should be a power of 2 greater than HIGH_PERIOD*2
#if WINDOW_LENGTH < HIGH_PERIOD*2
    #error WINDOW_LENGTH must be greater than HIGH_INDEX*2
#endif
#define BIT_PER_WORD    32
#define BIT_ARRAY_SIZE  (WINDOW_LENGTH/BIT_PER_WORD)	

#define THRESHOLD       (WINDOW_LENGTH/50)

#define SLICE 512
#define N_SLICES        ((SIGNAL_LENGTH-WINDOW_LENGTH)/SLICE)

#define SIGNAL_LENGTH 30000

static uint16_t signal_array[SIGNAL_LENGTH];
static uint32_t bit_array[BIT_ARRAY_SIZE];
static uint32_t acorr_array[WINDOW_LENGTH];


void convert_signal_to_bit(uint16_t *signal_array, uint32_t *bit_array)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		bit_array[i] = 0;
		for (int j = 0; j < BIT_PER_WORD; j++) {
			bit_array[i] |= (signal_array[i * BIT_PER_WORD + j] > 2048) ? (1 << j) : 0;
		}
	}
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
	for (int i = 0; i < BIT_ARRAY_SIZE-1; i++) {
		bit_array[i] = (bit_array[i] >> 1) | (bit_array[i + 1] << (BIT_PER_WORD - 1));
	}
    bit_array[BIT_ARRAY_SIZE-1] = bit_array[BIT_ARRAY_SIZE] >> 1;   
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
	if (end_bit > WINDOW_LENGTH) end_bit = WINDOW_LENGTH;
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

    memcpy(shifted_bit_array, bit_array, BIT_ARRAY_SIZE*sizeof(uint32_t));

    for (int i = 0; i < length; i++) {
        xor_bit_arrays(bit_array, shifted_bit_array, result_bit_array);
        acorr_array[i] = sum_bit_array(result_bit_array, i, i+HIGH_PERIOD);
        shift_bit_array(shifted_bit_array);
    }
}

int find_first_min_index(uint32_t *array, int length, int start_index)
{
    if (start_index == 0) start_index = 1;

    for (int i = start_index; i < length; i++) {
        if (array[i-1] >= array[i] && 
            array[i] <= array[i+1] &&
            array[i] <= THRESHOLD) {
            return i;
        }
    }
    return 0;
}

int read_signal_file(const char *filename, uint16_t  *signal, int length)
{
    int status;
    char line[256];
    FILE *file = fopen(filename, "r");
    if (file == NULL) {
        return -1;
    }
    for (int i = 0; i < length; i++) {
        status = fscanf(file, "%s", line);
        if (status == 0) {
            fclose(file);
            return -1;
        }
        signal[i] = (uint16_t)atoi(line);
    }
    fclose(file);
    return 0;
}

char *file_names[6] = {
    "E2_3S.cap",
    "A2_3S.cap",
    "D3_3S.cap",
    "G3_3S.cap",
    "B3_3S.cap",
    "E4_3S.cap"
};

int main() {
    int status;
    printf("testobp2\n");

    for (int i = 0; i < 6; i++) {
        status = read_signal_file(file_names[i], signal_array, SIGNAL_LENGTH);
        if (status != 0) {
            printf("Error reading file %s\n", file_names[i]);
            return -1;
        }
        for (int j = 0; j < N_SLICES; j++) {
            int start = j * SLICE;
            int end = start + WINDOW_LENGTH;
            convert_signal_to_bit(signal_array + start, bit_array);
            auto_diff_corr(bit_array, acorr_array, HIGH_PERIOD);
            int index = find_first_min_index(acorr_array, HIGH_PERIOD, LOW_PERIOD);

            printf("string: %1d  start: %5d  freq: %7.3f\n", i+1, start, (float)SAMPLING_FREQ/index);
        }
    }
}
