/**
 * @name autocorr
 * @details
 * Sample program to be used as a bass/guitar tuner using an autocorrelation algorythm
 * Designed for a Raspberry pi Pico (original) 
 * This program uses ADC0 to sample a biased audio signal (0-3.3V) that was analogicaly 
 * filtered by an 8 poles low pass filter set with a 1 KHz cut off frequency. 
 * 
 * @author Vincent Lacasse
 * @date   2025-04-11
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "data.h"

/*
 * Basse guitar string frequencies in Hz
 * E1   41.21
 * A1   55.00
 * D2   74.42
 * G2   98.00
 * 
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110.00
 * D3   146.83
 * G3   196.00
 * B3   246.94
 * E4   329.63
 */

// Using ADC Channel 0 (GPIO26)
#define ADC_CHANNEL 0

/**
 * Sampling frequency:
 * With the Pico internal 48 MHz clock, this DIV.INT will result in a 10 KHz sampling rate.
 * This should be enough to cover the highest frequency we want to measure, 
 * that is E4 on a guitar (329.63 Hz). Thus, 10 KHz is 15 times higher than the Nyquist frequency.
 * Note that the analog signal is filtered by a MAX7040 circuit (8 poles low pass filter) 
 * set up with a 1 KHz cut off frequency, in ordre to avoid signal aliasing.
 */
#define ADC_DIVINT  4799
#define SAMPLING_FREQUENCY 10000

/**
 * Signal acquisition:
 * At a sampling rate of 10 KHz, using 1024 samples will result in a sampling period of 102.4 ms.
 * Thus the actual refresh time will be approximatly 100 ms or one tenth of a second.
 * The buffer length is determined by the lowest frequency we want to measure. 
 * That is E1 - 41.21 Hz from the electric bass.
 * We actually have to measure at lower frequencies since we have to expect an E1 string that 
 * is mistuned at a lower frequency. If we allow the lowest frequency to be, say, 35 Hz
 * then a buffer length of 1024 @ 10 KHz will cover over 3 cycles of 35 Hz signal which 
 * should be enough.
 */
#define SIGNAL_LEN  1024

#define THRESHOLD_FACTOR      0.7
#define SET_THRESHOLD           0
#define SEARCH_UP_AND_EXCEEDED  1
#define SEARCH_DOWN             2
#define PEAK_FOUND              3

#define VALID    0
#define INVALID -1

typedef struct peak {
    int index;            // peak index in signal array
    double precise_index; // precise peak index
    double value;         // peak value
} peak_t;

uint16_t signal[SIGNAL_LEN];
int16_t mean;
float frequency;
int status;

// prototypes
int measure_frequency(uint16_t *signal, int n, float *frequency);
int acorr_find_first_peak(int16_t *signal, float threshold_factor, int n, int* index);
float find_precise_peak(int16_t *signal, int index);
uint16_t get_bias(uint16_t *signal, int n) ;
void remove_bias(uint16_t *signal_in, int16_t *signal_out, uint16_t bias, int n);

// main entry point
int main() {
    stdio_init_all();

        goto processing;

    // Init GPIO for analogue use: hi-Z, no pulls, disable digital input buffer.
    adc_gpio_init(26 + ADC_CHANNEL);

    adc_init();
    adc_select_input(ADC_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false    // Do not shift each sample 
    );

    // Set sample speed  
    adc_set_clkdiv(ADC_DIVINT);

    // Set up the DMA to start transferring data as soon as it appears in FIFO
    uint dma_chan = dma_claim_unused_channel(true);
    dma_channel_config cfg = dma_channel_get_default_config(dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&cfg, DREQ_ADC);

    dma_channel_configure(dma_chan, &cfg,
        signal,         // dst
        &adc_hw->fifo,  // src
        SIGNAL_LEN,     // transfer count
        true            // start immediately
    );

    // Start capture
    adc_run(true);
    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    dma_channel_wait_for_finish_blocking(dma_chan);
    adc_run(false);
    adc_fifo_drain();

processing:

    // Signal processing
    // status = measure_frequency(signal, SIGNAL_LEN, &frequency);
    status = measure_frequency(data, SIGNAL_LEN, &frequency);


    // Print measured frequency if valid
    if (status == VALID) {
        printf(" %6.2f\n", frequency);
    }
    else {
        printf(" INVALID\n");
    }
}

/**
 * @brief measure the main frequency of the signal using a autocorrelation algorythm
 * @param signal_in array containing the input signal
 * @param n         array length
 * @param frequency frequency measured. output.
 * @returns         VALID if frequency contains a valid value
 */
int measure_frequency(uint16_t *signal_in, int n, float *frequency)
{
    int status;
    int index;
    float precise_index = 0.0;
    uint16_t bias;
    int16_t  unbiased_signal[SIGNAL_LEN];

    bias = get_bias(signal_in, n);
    remove_bias(signal_in, unbiased_signal, bias, n);

    status = acorr_find_first_peak(unbiased_signal, THRESHOLD_FACTOR, n, &index);

    if (status == VALID) {
        precise_index = find_precise_peak(unbiased_signal, index);
        *frequency = SAMPLING_FREQUENCY / precise_index;
        return VALID;
    }
    return INVALID;
}



/**
 * @brief           find the precise location of a peak in signal given its index
 *                  based on the signal morphology
 * @param signal    array containing the signal
 * @param index     previously found peak index
 * @returns         peak position with greater precision
 */
float find_precise_peak(int16_t *signal, int index)
{
  float delta;

  delta = (signal[index-1] - signal[index+1]) * 0.5f /
          (signal[index-1] - (2.0f * signal[index]) + signal[index+1]);

  // value = (signal[index-1] + (2.0 * signal[index]) + signal[index+1]) / 4;

  return index + delta;
}


/**
 * @brief           Perform an autocorrelation on signal and find the index of first peak 
 *                  found in the signal. The peak must exceed signal[0]*threshold_factor. 
 * @param signal    array containing the signal
 * @param threshold_factor 
 *                  value between 0 and 1 by which signal[0] is multiplied
 *                  to obtain the detection threshold.
 * @param n         array length
 * @param index     signal index where the peak was found
 * @returns         true if a peak was found
 */
int acorr_find_first_peak(int16_t *signal, float threshold_factor, int n, int* index)
{
    int i, k;
    int state;
    int slope_positive;
    int threshold_exceeded;
    int64_t sum;
    int64_t previous_sum;
    int64_t threshold;
    
    state = SET_THRESHOLD;
    sum = 0;

    for(i = 0; i < n; i++) {

        previous_sum = sum;
        sum = 0;
        for(k = 0; k < n - i; k++)
            sum += signal[k] * signal[k+i];
    
        // Peak detection state machine
        slope_positive = (sum - previous_sum) > 0;
        threshold_exceeded = sum > threshold;

        if (state == SEARCH_DOWN && !slope_positive) {
            state = PEAK_FOUND;
            break;
        }
        if (state == SEARCH_UP_AND_EXCEEDED && slope_positive && threshold_exceeded) {
            state = SEARCH_DOWN;
        }
        if (state == SET_THRESHOLD) {
            threshold = (int64_t) (sum * threshold_factor);
            state = SEARCH_UP_AND_EXCEEDED;
        }
    }

    if (state == PEAK_FOUND) {
        *index = i;
        return VALID;
    }
    return INVALID;
}


/**
 * @brief compute the bias of an analog signal
 * @param signal  array containing the signal.
 * @param n       array length
 * @returns       the bias value
 */
uint16_t get_bias(uint16_t *signal, int n) 
{
    int32_t bias = 0;
    for (int i = 0; i < n; i++)
        bias += signal[i];
    return (uint16_t) (bias / n);
}


/**
 * @brief remove bias from an analog signal
 * @param signal_in  input array containing the signal to remove the bias from.
 * @param signal_out output array containing the signal from which the bias was removed. 
 * @param bias       value to remove from the input signal
 * @param n          array length
 */
void remove_bias(uint16_t *signal_in, int16_t *signal_out, uint16_t bias, int n) 
{
    for (int i = 0; i < n; i++) 
        signal_out[i] = (int16_t)signal_in[i] - (int16_t)bias;
}


/**
 * @brief   estimate the power of the input signal
 * @param signal        array containing the input signal
 * @param start_index   starting index 
 * @param stop_index    stoping index
 * @returns             signal power
 * @note    the signal shall not include any bias.
 */
float estimate_signal_power(float *signal, int start_index, int stop_index)
{
  float power = 0.0;
  for (int i = start_index; i <= stop_index; i++) power += signal[i] * signal[i];
  return power / (stop_index - start_index + 1);  
}


