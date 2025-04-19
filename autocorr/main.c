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

// #include "data.h"
// #include "E.h"

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

// maximum and minimum detection frequencies
#define MAX_FREQ    400     // in Hz
#define MIN_FREQ    35      // in Hz

// peak detection search limits in signal autocorrelation
#define MIN_COUNT   (SAMPLING_FREQUENCY/MAX_FREQ)
#define MAX_COUNT   (SAMPLING_FREQUENCY/MIN_FREQ)

// variable thresholds and limits for peak detection
// i.e. I have observed that the detection threshold should be 
// set at a lower value when seaching lower frequency.
// These values were set empirically.
#define HIGH_THRES      0.80
#define MID_THRES       0.60
#define LOW_THRES       0.40
#define MID_MAX_FREQ    190     // in Hz
#define LOW_MAX_FREQ    100     // in Hz
#define MID_MIN_COUNT   (SAMPLING_FREQUENCY/MID_MAX_FREQ)
#define LOW_MIN_COUNT   (SAMPLING_FREQUENCY/LOW_MAX_FREQ)

#define SET_THRESHOLD           0
#define SEARCH_UP_AND_EXCEEDED  1
#define SEARCH_DOWN             2
#define PEAK_FOUND              3

#define VALID        0
#define NO_PEAK     -1
#define TOO_HIGH    -2
#define TOO_LOW     -3


// prototypes
int init_data_acquisition(uint *dma_chan, dma_channel_config *cfg);
int get_signal(uint dma_chan, dma_channel_config *cfg, uint16_t *signal);
int measure_frequency(uint16_t *signal, float *frequency);
int acorr_find_first_peak(int16_t *signal, float *acorr, int* index);
float find_precise_peak(float *signal, int index);
uint16_t get_bias(uint16_t *signal) ;
void remove_bias(uint16_t *signal_in, int16_t *signal_out, uint16_t bias);

// main entry point
int main() 
{
    float frequency;
    int status;
    uint dma_chan;
    dma_channel_config cfg;
    uint16_t signal[SIGNAL_LEN];

    stdio_init_all();

    init_data_acquisition(&dma_chan, &cfg);

    while (1) 
    {
        get_signal(dma_chan, &cfg, signal);

        status = measure_frequency(signal, &frequency);

        // Print measured frequency if valid
        switch(status) {
            case VALID:
            printf(" %6.2f\n", frequency);
            break;

            case NO_PEAK:
            printf(" NO PEAK\n");
            break;

            case TOO_LOW:
            printf(" TOO LOW\n");
            break;

            case TOO_HIGH:
            printf(" TOO HIGH\n");
            break;
        }
    }
}


/**
 * @brief           Initialize pico data acquisition
 * @param dma_chan  pointer to dma channel assigned to ADC transfers. Output
 * @param cfg       pointer to dma channel configuration. Output
 * @returns         0
 */
int init_data_acquisition(uint *dma_chan, dma_channel_config *cfg) 
{
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
    *dma_chan = dma_claim_unused_channel(true);
    *cfg = dma_channel_get_default_config(*dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(cfg, DMA_SIZE_16);
    channel_config_set_read_increment(cfg, false);
    channel_config_set_write_increment(cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(cfg, DREQ_ADC);
    
    return 0;
}

/**
 * @brief           Digitize n samples with ADC and place them in array signal
 * @param dma_chan  dma channel assigned to ADC transfers.
 * @param cfg       pointer to dma channel configuration. cfg is modified.
 * @param signal    array filled with digitized samples
 * @returns         0
 */
int get_signal(uint dma_chan, dma_channel_config *cfg, uint16_t *signal)
{
    dma_channel_configure(dma_chan, cfg,
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

    return 0;
}


/**
 * @brief measure the main frequency of the signal using a autocorrelation algorythm
 * @param signal_in array containing the input signal
 * @param frequency frequency measured. output.
 * @returns         VALID if frequency contains a valid value
 */
int measure_frequency(uint16_t *signal_in, float *frequency)
{
    int status;
    int index;
    float precise_index = 0.0;
    uint16_t bias;
    int16_t  unbiased_signal[SIGNAL_LEN];
    float    acorr[SIGNAL_LEN];

    bias = get_bias(signal_in);
    remove_bias(signal_in, unbiased_signal, bias);

    status = acorr_find_first_peak(unbiased_signal, acorr, &index);

    if (status == VALID) {
        precise_index = find_precise_peak(acorr, index);
        *frequency = SAMPLING_FREQUENCY / precise_index;
    }

    return status;
}


/**
 * @brief           Perform an autocorrelation on signal and find the index of first peak 
 *                  found in the signal. The peak must exceed signal[0]*threshold_factor. 
 * @param signal    array containing the signal
 * @param acorr     array containing the autocorrelation of the signal up to index+1 only
 * @param threshold_factor 
 *                  value between 0 and 1 by which signal[0] is multiplied
 *                  to obtain the detection threshold.
 * @param index     signal index where the peak was found
 * @returns         true if a peak was found
 */
int acorr_find_first_peak(int16_t *signal, float *acorr, int* index)
{
    int i, k;
    int state;
    int slope_positive;
    int threshold_exceeded;
    int64_t sum;
    int64_t sum_at_index0;
    int64_t previous_sum;
    int64_t threshold;
    
    state = SET_THRESHOLD;
    sum = 0;
    sum_at_index0 = 1000;  // arbritary starting value, will be overwritten.

    for(i = 0; i < MAX_COUNT; i++) {

        // compute autocorrelation
        previous_sum = sum;
        sum = 0;
        for(k = 0; k < MAX_COUNT - i; k++)
            sum += signal[k] * signal[k+i];
    
        // record autocorrelation result
        acorr[i] = (float) sum / sum_at_index0;

        // set peak detection flags
        slope_positive = (sum - previous_sum) > 0;
        threshold_exceeded = sum > threshold;

        // peak detection state machine
        if (state == SEARCH_DOWN && !slope_positive) {
            state = PEAK_FOUND;
            break;
        }
        if (state == SEARCH_UP_AND_EXCEEDED && slope_positive && threshold_exceeded) {
            state = SEARCH_DOWN;
        }
        if (state == SET_THRESHOLD) {
            sum_at_index0 = sum;
            acorr[0] = 0.0f;  // overwrite acorr[0] on loop's first iteration
            threshold = (int64_t) (sum_at_index0 * HIGH_THRES);
            state = SEARCH_UP_AND_EXCEEDED;
        }
        if (i == MID_MIN_COUNT) {
            threshold = (int64_t) (sum_at_index0 * MID_THRES);
        }
        if (i == LOW_MIN_COUNT) {
            threshold = (int64_t) (sum_at_index0 * LOW_THRES);
        }
    }

    if (state == PEAK_FOUND) {
        if (i < MIN_COUNT)  return TOO_HIGH;
        if (i >= MAX_COUNT) return TOO_LOW;
        *index = i - 1;
        return VALID;
    }
    return NO_PEAK;
}


/**
 * @brief           find the precise location of a peak in signal given its index
 *                  based on the signal morphology
 * @param signal    array containing the signal
 * @param index     previously found peak index
 * @returns         peak position with greater precision
 */
float find_precise_peak(float *signal, int index)
{
  float delta;

  delta = (signal[index-1] - signal[index+1]) * 0.5f /
          (signal[index-1] - (2.0f * signal[index]) + signal[index+1]);

  // value = (signal[index-1] + (2.0 * signal[index]) + signal[index+1]) / 4;

  return index + delta;
}


/**
 * @brief compute the bias of an analog signal
 * @param signal  array containing the signal.
 * @returns       the bias value
 */
uint16_t get_bias(uint16_t *signal) 
{
    int32_t bias = 0;
    for (int i = 0; i < SIGNAL_LEN; i++)
        bias += signal[i];
    return (uint16_t) (bias / SIGNAL_LEN);
}


/**
 * @brief remove bias from an analog signal
 * @param signal_in  input array containing the signal to remove the bias from.
 * @param signal_out output array containing the signal from which the bias was removed. 
 * @param bias       value to remove from the input signal
 */
void remove_bias(uint16_t *signal_in, int16_t *signal_out, uint16_t bias) 
{
    for (int i = 0; i < SIGNAL_LEN; i++) 
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


