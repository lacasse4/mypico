/**
 * @name tuner_yin
 * @details
 * simple F0 detector using YIN algorithm taken from PitchTrack github
 * (https://github.com/PitchTrack/PitchTrack)
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-18
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>

#include "pico/stdio.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/dma.h"

#include "pitch.h"
#include "../pitch/_pitch_yin.h"

// Using ADC Channel 0 (GPIO26)
#define ADC_CHANNEL 0

/**
 * Sampling frequency:
 * Pico is using an internal 48 MHz adc clock (clk_adc)
 */
#define PICO_CLK_ACD        48000000    // in Hz
#define SAMPLING_FREQUENCY  32000       // in Hz
#define ADC_DIVINT          ((PICO_CLK_ACD/SAMPLING_FREQUENCY)-1)

/**
 * Signal acquisition:
 */
#define SIGNAL_LEN  2048

// maximum and minimum detectable frequencies
#define MAX_FREQ    500     // in Hz
#define MIN_FREQ    35      // in Hz

/*
 * Small utilities to show an "alive" signal and for debugging
 */

// prototypes
int init_data_acquisition(uint *dma_chan, dma_channel_config *cfg);
int get_signal(uint dma_chan, dma_channel_config cfg, uint16_t *signal);
float get_bias(uint16_t *signal) ;
void remove_bias(uint16_t *signal_in, float *signal_out, float bias);


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
 */

 int main() {
    uint32_t start_time = 0;
    uint32_t elapsed_time, now;

    uint dma_chan;
    dma_channel_config cfg;
    uint16_t signal[SIGNAL_LEN];
    float unbiased_signal[SIGNAL_LEN];
    float bias;

    // Initialise YIN object
	PitchYINObj pitch=NULL;
	
	int samplate=SAMPLING_FREQUENCY;
	float lowFre=MIN_FREQ;
	float highFre=MAX_FREQ;

	// max radix2Exp is 2048 on Pico
	int radix2Exp=11; // 2048
	int slideLength=(1<<radix2Exp)/4;
	int autoLength=1024;
	int len = SIGNAL_LEN; 

    int isContinue=0;
	float *freArr=NULL;

    // initialize printing to console
    stdio_init_all();

    // Inititialize on board LED (live signal)
    init_led();   
    
    // initialize ADC
    init_data_acquisition(&dma_chan, &cfg);

    // initialize pitch detection
	pitchYINObj_new(&pitch,
				&samplate, &lowFre, &highFre,
				&radix2Exp, &slideLength, &autoLength,
				&isContinue);
	freArr=__vnew(10, NULL);

    while (true) {

        sleep_ms(5);

        // acquire SIGNAL_LEN samples in 'signal'
        get_signal(dma_chan, cfg, signal);

        bias = get_bias(signal);
        remove_bias(signal, unbiased_signal, bias);

        start_time = time_us_32();
        pitchYINObj_pitch(pitch,unbiased_signal,len,freArr, NULL, NULL);
        now = time_us_32();
        elapsed_time =  now - start_time;

        printf("  f: %7.3lf  et: %ld   \n", freArr[0], elapsed_time);
        freArr[0] = 0.0;
    }

    free(freArr);
	pitchYINObj_free(pitch);

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
 * @brief compute the bias of an analog signal
 * @param signal  array containing the signal.
 * @returns       the bias value
 */
float get_bias(uint16_t *signal) 
{
    int32_t bias = 0;
    for (int i = 0; i < SIGNAL_LEN; i++)
        bias += signal[i];
    return ((float)bias / SIGNAL_LEN);
}


/**
 * @brief remove bias from an analog signal
 * @param signal_in  input array containing the signal to remove the bias from.
 * @param signal_out output array containing the signal from which the bias was removed. 
 * @param bias       value to remove from the input signal
 */
void remove_bias(uint16_t *signal_in, float *signal_out, float bias) 
{
    for (int i = 0; i < SIGNAL_LEN; i++) 
        signal_out[i] = signal_in[i] - bias;
}


