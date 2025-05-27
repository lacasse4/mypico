#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
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

#define AC_HIGH_THRES1  (WINDOW_LENGTH/11)
#define AC_LOW_THRES1   0.25
#define AC_HIGH_THRES2  (WINDOW_LENGTH/8)
#define AC_LOW_THRES2   (WINDOW_LENGTH/32)

#define BIT_THRESHOLD   (WINDOW_LENGTH/5)

// #define SIGNAL_LENGTH   (31*1024) // 3 seconds of data
#define SIGNAL_LENGTH   30000 // 3 seconds of data

// #define PEAK_LIST_LENGTH  5   // maximum number of peaks in the peak list

#define SLICE           512     // the input signal is processed each SLICE samples
#if SLICE > WINDOW_LENGTH
    #error SLICE must be less than or equal to WINDOW_LENGTH
#endif
#define N_SLICES        ((SIGNAL_LENGTH-WINDOW_LENGTH)/SLICE + 1)


void remove_signal_offset(uint16_t *array_in, int16_t *array_out, int length)
{
    uint32_t sum = 0;
    for (int i = 0; i < length; i++) {
        sum += array_in[i];
    }
    uint16_t average = sum / length;

    for (int i = 0; i < length; i++) {
        array_out[i] = (int16_t)array_in[i] - average;
    }
}

bool convert_signal_to_bit(int16_t *array_in, uint32_t *bit_array_out)
{
    bool is_one;
    int sum_ones = 0;
    int sum_zeros = 0;

	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
        bit_array_out[i] = 0;
		for (int j = 0; j < BIT_PER_WORD; j++) {
            is_one = (array_in[i * BIT_PER_WORD + j] > 0);
            if (is_one) {
                bit_array_out[i] |= (1 << j);
                sum_ones++;
            }
            else {
                sum_zeros++;
            }
 		}
	}

    // return abs(sum_ones - sum_zeros) < BIT_THRESHOLD;
    return true; //abs(sum_ones - sum_zeros) < BIT_THRESHOLD;
}

void print_bit_array(uint32_t *bit_array_in)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		for (int j = 0; j < BIT_PER_WORD; j++) {
			int bit = (bit_array_in[i] & (1 << j)) ? 1 : 0;
			printf("%d", bit);		
		}
	}
    printf("\n");
}

void shift_bit_array(uint32_t *bit_array_in_out)
{
	for (int i = 0; i < BIT_ARRAY_SIZE-1; i++) {
		bit_array_in_out[i] = (bit_array_in_out[i] >> 1) | (bit_array_in_out[i + 1] << (BIT_PER_WORD - 1));
	}
    bit_array_in_out[BIT_ARRAY_SIZE-1] = bit_array_in_out[BIT_ARRAY_SIZE] >> 1;   
}

void xor_bit_arrays(uint32_t *bit_array1_in, uint32_t *bit_array2_in, uint32_t *bit_array1_out)
{
	for (int i = 0; i < BIT_ARRAY_SIZE; i++) {
		bit_array1_out[i] = bit_array1_in[i] ^ bit_array2_in[i];
	}
}

uint16_t sum_bit_array(uint32_t *bit_array_in, int start_bit, int end_bit)
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
		uint32_t v = bit_array_in[i];
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

void auto_diff_corr(uint32_t *bit_array_in, uint16_t *acorr_array_out, int length)
{
    uint32_t shifted_bit_array[BIT_ARRAY_SIZE];
    uint32_t result_bit_array[BIT_ARRAY_SIZE];

    memcpy(shifted_bit_array, bit_array_in, BIT_ARRAY_SIZE*sizeof(uint32_t));

    for (int i = 0; i < length; i++) {
        xor_bit_arrays(bit_array_in, shifted_bit_array, result_bit_array);
        acorr_array_out[i] = sum_bit_array(result_bit_array, i, i+HIGH_PERIOD);
        shift_bit_array(shifted_bit_array);
    }
}


#define START 0
#define DESCENDING 1
#define FLAT 2
#define ASCENDING 3

/*
float find_first_min_index(uint16_t *array_in, int length, int start_index)
{
    int state = 0;
    int flat_count = 0;
    if (start_index == 0) start_index = 1;

    for (int i = start_index; i < length; i++) {
        if (array_in[i-1] < array_in[i]) {    // ascending
            if (state == DESCENDING && array_in[i-1] <= AC_THRESHOLD) {
                return i - 1;             // was descending, now ascending -> done searching
            } 
            else if (state == FLAT && array_in[i-1] <= AC_THRESHOLD) {
                return i - 1 - (float)flat_count/2.0;
            } 
            else {
                state = ASCENDING;      // set ascending state
            }
        } 
        else if (array_in[i-1] > array_in[i]) {  // descending
            state = DESCENDING;
        }
        else if (array_in[i-1] == array_in[i]) { // flat
            if (state == DESCENDING && array_in[i-1] <= AC_THRESHOLD) {
                flat_count = 1;
                state = FLAT;
            } 
            else if (state == FLAT) {
                flat_count++;
            } 
            else {
                flat_count = 0;
                state = START;
            }
        }
    }
    return 0.0;
}
*/


int find_next_max(uint16_t *array_in, int length, int start_index)
{
    for (int i = start_index; i < length - 1; i++) {
        if (array_in[i] > array_in[i + 1]) {
            return i;
        }
    }
    return -1; // no maximum found
}

int find_next_min(uint16_t *array_in, int length, int start_index)
{
    for (int i = start_index; i < length - 1; i++) {
        if (array_in[i] < array_in[i + 1]) {
            return i;
        }
    }
    return -1; // no minimum found
}

#define GAP 20 // minimum gap between peaks in samples
float get_F0_period(uint16_t *array_in, int length, int start_index)
{
    int imax1 = find_next_max(array_in, length, start_index);
    uint16_t max1_value = array_in[imax1];
    printf("| i: %3d ", imax1);
    printf("v: %3u | ", max1_value);
    if (max1_value < AC_HIGH_THRES1) return 0.0; // maximum too low
    if (imax1 < 0) return 0.0; // no maximum found

    int imin1 = find_next_min(array_in, length, imax1 + GAP);
    printf("i: %3d ", imin1);
    printf("v: %3u | ", array_in[imin1]);
    if (array_in[imin1] > AC_LOW_THRES1*max1_value) return 0.0; // minimum too high
    if (imin1 < 0) return 0.0; // no minimum found

    int imax2 = find_next_max(array_in, length, imin1 + GAP);
    printf("i: %3d ", imax2);
    printf("v: %3u | ", array_in[imax2]);
    if (array_in[imax2] < AC_HIGH_THRES2) return 0.0; // second maximum too low
    if (imax2 < 0) return 0.0; // no second maximum found

    int imin2 = find_next_min(array_in, length, imax2 + GAP);
    printf("i: %3d ", imin2);
    printf("v: %3u | ", array_in[imin2]);
    if (array_in[imin2] > AC_LOW_THRES2) return 0.0; // minimum after second maximum too high
    if (imin2 < 0) return 0.0; // no minimum found after second maximum
    return imin2;
}

int find_max(uint16_t *array_in, int length, int start_index)
{
    int imax = -1;
    uint16_t max_value = 0;
    for (int i = start_index; i < length; i++) {
        if (array_in[i] > max_value) {
            max_value = array_in[i];
            imax = i;
        }
    }
    return imax;
}

int find_min(uint16_t *array_in, int length, int start_index)
{
    int imin = -1;
    uint16_t min_value = UINT16_MAX;
    for (int i = start_index; i < length; i++) {
        if (array_in[i] < min_value) {
            min_value = array_in[i];
            imin = i;
        }
    }
    return imin;
}

int find_first_min_bellow(uint16_t *array_in, int length, int start_index, uint16_t threshold)
{
    for (int i = start_index; i < length - 1; i++) {
        if (array_in[i] > threshold) {
            continue; // skip values above threshold
        }
        if (array_in[i] < array_in[i + 1]) {
            return i;
        }
    }
    return -1; // no minimum found
}

float get_F0_period2(uint16_t *array_in, int length, int start_index)
{
    int imax = find_next_max(array_in, length, start_index);
    if (imax < 0) return 0.0; // no maximum found
    int max_value = array_in[imax];

    int imin1 = find_first_min_bellow(array_in, length, imax+GAP, max_value * AC_LOW_THRES1);
    if (imin1 < 0) return 0.0; // no minimum found below threshold
    int min1_value = array_in[imin1];

    int imin2 = find_first_min_bellow(array_in, length, imin1+GAP, max_value * AC_LOW_THRES1);
    if (imin2 < 0) return (float)imin1; // no second minimum found below threshold
    int min2_value = array_in[imin2];
    if (min2_value < min1_value) return (float)imin2; 
    return (float)imin1; // return the first minimum if the second is not lower
}

/*
bool find_min_peaks(uint16_t *array_in, int length, int start_index, float *peak_list_out, int *peak_count)
{
    int flat_count = 0;
    int state = START; // 0: start, 1: descending, 2: flat, 3: ascending
    *peak_count = 0;
    if (start_index == 0) start_index = 1;

    for (int i = 0; i < PEAK_LIST_LENGTH; i++) {
        peak_list_out[i] = -1.0; // initialize peak list
    }

    for (int i = start_index; i < length; i++) {
        if (array_in[i-1] < array_in[i]) {    // ascending
            if (state == DESCENDING) {
                peak_list_out[*peak_count] = i - 1; // was descending, now ascending -> found a peak
                (*peak_count)++;
                if (*peak_count >= PEAK_LIST_LENGTH) {
                    return true; // peak list is full
                } 
            else if (state == FLAT) {
                peak_list_out[*peak_count] = i - 1 - (float)flat_count/2.0; // was flat, now ascending -> found a peak
                (*peak_count)++;
                if (*peak_count >= PEAK_LIST_LENGTH) {
                    return true; // peak list is full
                }
            }
            else {
                state = ASCENDING;      // set ascending state
            }
        } 
        else if (array_in[i-1] > array_in[i]) {  // descending
            state = DESCENDING;
        }
        else if (array_in[i-1] == array_in[i]) { // flat
            if (state == DESCENDING) {
                flat_count = 1;
                state = FLAT;
            } 
            else if (state == FLAT) {
                flat_count++;
            } 
            else {
                flat_count = 0;
                state = START;
            }
        }
    }
    return true;
}
*/

int read_signal_file(const char *filename, uint16_t  *array_out, int length)
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
        array_out[i] = (uint16_t)atoi(line);
    }
    fclose(file);
    return 0;
}

char *file_names[6] = {
    "/Users/vincent/Documents/data/E2.cap",
    "/Users/vincent/Documents/data/A2.cap",
    "/Users/vincent/Documents/data/D3.cap",
    "/Users/vincent/Documents/data/G3.cap",
    "/Users/vincent/Documents/data/B3.cap",
    "/Users/vincent/Documents/data/E4.cap"
};

char *file_names_out[6] = {
    "/Users/vincent/Documents/data/E2acorr.cap",
    "/Users/vincent/Documents/data/A2acorr.cap",
    "/Users/vincent/Documents/data/D3acorr.cap",
    "/Users/vincent/Documents/data/G3acorr.cap",
    "/Users/vincent/Documents/data/B3acorr.cap",
    "/Users/vincent/Documents/data/E4acorr.cap"
};


float periods[6][2] = {
    82.41, // E2
    110.00, // A2
    146.83, // D3
    196.00, // G3
    246.94, // B3
    329.63  // E4
};

int main() {
    FILE *file;
    static uint16_t signal_array_in[SIGNAL_LENGTH];
    static int16_t  signal_array_unbiased[SIGNAL_LENGTH];
    static uint32_t bit_array[BIT_ARRAY_SIZE];
    static uint16_t acorr_array[WINDOW_LENGTH];
    static int16_t signal_slice[WINDOW_LENGTH];
    // static float peak_list[PEAK_LIST_LENGTH];

    printf("testobp2\n");

    for (int i = 0; i < 6; i++) {
        if (read_signal_file(file_names[i], signal_array_in, SIGNAL_LENGTH)) {
            printf("Error reading file %s\n", file_names[i]);
            return -1;
        }
        file = fopen(file_names_out[i], "w");
        if (file == NULL) {
            printf("Error opening file %s for writing\n", file_names[i]);
            return -1;
        }

        remove_signal_offset(signal_array_in, signal_array_unbiased, SIGNAL_LENGTH);
        
        for (int j = 0; j < N_SLICES; j++) {

            printf("s: %3d j: %3d ", i+1, j);

            int start = j * SLICE;
            memcpy(signal_slice, signal_array_unbiased + start, WINDOW_LENGTH * sizeof(int16_t));
            bool is_ok = convert_signal_to_bit(signal_slice, bit_array);
            auto_diff_corr(bit_array, acorr_array, HIGH_PERIOD);

            for (int k = 0; k < HIGH_PERIOD; k++) {
                fprintf(file, "%5u ", acorr_array[k]);
            }
            fprintf(file, "\n");
            
            // float index = find_first_min_index(acorr_array, HIGH_PERIOD, LOW_PERIOD);
            float index = get_F0_period2(acorr_array, HIGH_PERIOD, LOW_PERIOD);
            float frequency;
            if (!is_ok || index == 0) {
                frequency = 0;
            } else {
                frequency = (float)SAMPLING_FREQ / index; 
            }

            printf("i: %5.1f f: %6.2f ", index, frequency);
            printf("\n");

        }
        fclose(file);
    }
}
