/**
 * @name tuner_acorr
 * @details
 * Simple bass/guitar tuner using an autocorrelation algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - uses ADC0 to sample a biased audio signal (0-3.3V).
 *  - outputs to console
 * 
 * @note  It is assumed that the input signal to ADC0 is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-04-11
 */

#include <stdio.h>
#include <assert.h>

// pico libraries
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

// various signal processing utilities 
#include "accur.h"
#include "alpha.h"
#include "limit.h"
#include "calibrate.h"

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

// test data (analogicaly filtered)
// #include "data.h" (100 Hz)
// #include "E.h"    (from guitar)

/*
 * Calibration data gathered manually.
 * It's to adjust the frequency for an observed
 * but unexplained offset. A linear calibration is done
 * via a least square estimation of the folling data:
 *  
 * |   fin  |  fout  |
 * -------------------
 * |  71.55 |  70.00 |  
 * | 101.39 | 100.00 | 
 * | 201.08 | 200.00 | 
 * | 301.00 | 300.00 |
 *  
 */
#define N_CALIB  4
double x[] = { 71.55, 101.39, 201.08, 301.00 };
double y[] = { 70.00, 100.00, 200.00, 300.00 };

// Using ADC Channel 0 (GPIO26)
#define ADC_CHANNEL 0

/**
 * Sampling frequency:
 * uning the Pico internal 48 MHz adc clock (clk_adc), DIV.INT will result in a 10 KHz 
 * sampling frequency.
 * This should be enough to cover the highest frequency to be measured, that is E4 
 * on a guitar (329.63 Hz). Thus, 10 KHz is 15 times higher than the Nyquist frequency.
 * Note that the analog signal is filtered by a MAX7040 circuit (8 poles low pass filter) 
 * set up with a 1 KHz cut off frequency, in ordre to avoid signal aliasing.
 */
#define PICO_CLK_ACD        48000000    // in Hz
#define SAMPLING_FREQUENCY  10000       // in Hz
#define ADC_DIVINT          ((PICO_CLK_ACD/SAMPLING_FREQUENCY)-1)

/**
 * Signal acquisition:
 * With a 10 KHz sampling frequency, 1024 samples in a row are captured.
 * This results in a sampling period of 102.4 ms.
 * Thus, the current refresh time is approximatly a tenth of a second.
 * The buffer length is determined by the lowest frequency to be measured, that is E1
 * or 41.21 Hz from the electric bass.
 * We actually have to measure at lower frequencies since we have to expect 
 * an E1 string that is mistuned at a lower frequency. If we allow the lowest 
 * frequency to be, say, 35 Hz then a buffer length of 1024 @ 10 KHz will cover 
 * over 3 cycles of 35 Hz signal which should be more than enough.
 */
#define SIGNAL_LEN  1024

// maximum and minimum detectable frequencies
#define MAX_FREQ    400     // in Hz
#define MIN_FREQ    35      // in Hz

// peak detection search limits in acorr_find_first_peak()
#define MIN_COUNT   (SAMPLING_FREQUENCY/MAX_FREQ)
#define MAX_COUNT   (SAMPLING_FREQUENCY/MIN_FREQ)

// Variable thresholds and limits for peak detection
// i.e. I have observed that the detection threshold should be 
// set at a lower value when seaching at lower frequencies.
// These values were set empirically.
#define HIGH_THRES      0.80
#define MID_THRES       0.60
#define LOW_THRES       0.40
#define MID_MAX_FREQ    190     // in Hz
#define LOW_MAX_FREQ    100     // in Hz
#define MID_MIN_COUNT   (SAMPLING_FREQUENCY/MID_MAX_FREQ)
#define LOW_MIN_COUNT   (SAMPLING_FREQUENCY/LOW_MAX_FREQ)

// Minimum signal power to trigger peak detection
#define MIN_POWER       50000

// states for acorr_find_first_peak() state machine
#define SET_THRESHOLD           0
#define SEARCH_UP_AND_EXCEEDED  1
#define SEARCH_DOWN             2
#define PEAK_FOUND              3

// acorr_find_first_peak() return statuses
#define VALID        0
#define LOW_COUNT   -1
#define HIGH_COUNT  -2
#define LOW_POWER   -3

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA       0.05

// Accuracy target for accuracy and precision measurments (accur.c)
#define ACCURACY_TARGET 200

// A non-linear filter (limit.c) is used to discard output frequencies 
// that are to far from the previously measured frequency.
// MAX_FREQ_PCT is that maximum percentage allowed from the previous measurment.
#define MAX_FREQ_PCT    0.10


// for debugging
int global_index;
int counter = 0;
uint32_t start_time;
uint32_t gap;
double global_power;
double global_precise_index;


// prototypes
int init_data_acquisition(uint *dma_chan, dma_channel_config *cfg);
int get_signal(uint dma_chan, dma_channel_config cfg, uint16_t *signal);
int measure_frequency(uint16_t *signal, double *frequency);
int acorr_find_first_peak(int16_t *signal, double *acorr, int* index);
double find_precise_peak(double *signal, int index);
uint16_t get_bias(uint16_t *signal) ;
void remove_bias(uint16_t *signal_in, int16_t *signal_out, uint16_t bias);
double estimate_signal_power(int16_t *signal);


// main entry point
int main() 
{
    int detection_status;
    int statistic_status;

    double accuracy;
    double precision;
    double frequency;
    double filtered_frequency;
    double corrected_frequency;

    uint dma_chan;
    dma_channel_config cfg;
    uint16_t signal[SIGNAL_LEN];
    accur_t *accur;
    alpha_t *alpha;
    limit_t *limit;
    calibrate_t *calib; 

    

    // for printing to console
    stdio_init_all();

    // create an accuracy and precision measurment object 
    accur = create_accur(10, ACCURACY_TARGET);
    assert(accur);

    // create an IIR filter object
    alpha = create_alpha(ALPHA);
    assert(alpha);

    // create a non-linear filtering object 
    limit = create_limit(MAX_FREQ_PCT, 0.0);
    assert(limit);

    // create a calibration object
    // initialize it with calibration data gathered manually
    calib = create_calibrate(N_CALIB, x, y);
    assert(calib);
    
    // initialize ADC
    init_data_acquisition(&dma_chan, &cfg);

    while (1) 
    {
        start_time = time_us_32();                          // debug

        // acquire SIGNAL_LEN samples in 'signal'
        get_signal(dma_chan, cfg, signal);

        // measure signal fundamental frequency
        detection_status = measure_frequency(signal, &frequency);

        gap = time_us_32() - start_time;                    // debug

        // printf("  sec: %5.3lf  pow: %6.0lf", (double)gap/imit->status = LIMIT_RESET;1000000, global_power);

        printf("%5d", counter++);                           // debug

        // print measured frequency if valid
        if (detection_status == VALID) {

            printf("  frq: %6.2lf", frequency);
            printf("  idx: %d", global_index);              // debug
            printf("  pdx: %6.2lf", global_precise_index);  // debug

            if (limit_next(limit, frequency)) { 
                filtered_frequency = alpha_filter(alpha, frequency);
                corrected_frequency = calibrate_getY(calib, filtered_frequency);
                printf("  flt: %6.2lf", corrected_frequency);

                statistic_status = accur_add(accur, corrected_frequency);
                if (statistic_status == ACCUR_OK) {
                    statistic_status = accur_results(accur, &accuracy, &precision);
                    printf("  acc: % -4.2lf", accuracy);
                    printf("  prc: %4.2lf", precision);
                }
            }
            else {
                alpha_reset(alpha);
                accur_flush(accur);
            }
        }
        else {
            alpha_reset(alpha);
            accur_flush(accur);
        }
        printf("\n");
    }

    release_accur(accur);
    release_alpha(alpha);
    release_limit(limit);
    release_calibrate(calib);
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
        false,   // ERR bit disabled
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
int get_signal(uint dma_chan, dma_channel_config cfg, uint16_t *signal)
{
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

    return 0;
}


/**
 * @brief measure the main frequency of the signal using a autocorrelation algorythm
 * @param signal_in array containing the input signal
 * @param frequency frequency measured. output.
 * @returns         VALID if frequency contains a valid value
 */
int measure_frequency(uint16_t *signal_in, double *frequency)
{
    int status;
    int index;
    double precise_index = 0.0;
    double power;
    uint16_t bias;
    int16_t  unbiased_signal[SIGNAL_LEN];
    double   acorr[SIGNAL_LEN];

    bias = get_bias(signal_in);
    remove_bias(signal_in, unbiased_signal, bias);

    power = estimate_signal_power(unbiased_signal);
    global_power = power;
    if (power < MIN_POWER) return LOW_POWER;

    status = acorr_find_first_peak(unbiased_signal, acorr, &index);
    global_index = index;

    *frequency = 0.0;
    if (status == VALID) {
        precise_index = find_precise_peak(acorr, index);
        global_precise_index = precise_index;
        *frequency = SAMPLING_FREQUENCY / precise_index;
    }

    return status;
}


/**
 * @brief           Perform an autocorrelation on signal and find the index of first peak 
 *                  found in the signal. The peak must exceed signal[0]*threshold_factor. 
 * @param signal    array containing the signal
 * @param acorr     array containing the signal's normalized autocorrelation up to index+1 only
 * @param index     signal index where the peak was found
 * @returns         true if a peak was found
 */
int acorr_find_first_peak(int16_t *signal, double *acorr, int *index)
{
    int i, k;
    int state;
    int slope_positive;
    int threshold_exceeded;
    int64_t sum;
    int64_t sum_at_index0;
    int64_t previous_sum;
    int64_t threshold;
    
    *index = 0;
    state = SET_THRESHOLD;
    sum = 0;
    sum_at_index0 = 1;  // arbritary starting value to avoid division by 0, will be overwritten.

    for (i = 0; i < MAX_COUNT; i++) {

        // compute autocorrelation
        previous_sum = sum;
        sum = 0;
        for(k = 0; k < MAX_COUNT - i; k++)
            sum += signal[k] * signal[k+i];
    
        // record autocorrelation result
        acorr[i] = (double) sum / sum_at_index0;

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
            acorr[0] = 1.0;     // accor is normalized
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
        *index = i - 1;
        if (*index < MIN_COUNT) return LOW_COUNT;
        return VALID;
    }
    return HIGH_COUNT;
}


/**
 * @brief           find the precise location of a peak in signal given its index
 *                  based on the signal morphology
 * @param signal    array containing the signal
 * @param index     previously found peak index
 * @returns         peak position with greater precision
 */
double find_precise_peak(double *signal, int index)
{
  double delta;

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
 * @returns             signal power
 * @note    the signal shall not include any bias.
 */
double estimate_signal_power(int16_t *signal)
{
    int64_t power = 0.0;
    for (int i = 0; i < SIGNAL_LEN; i++) 
        power += signal[i] * signal[i];
    return (double)power / SIGNAL_LEN;  
}


