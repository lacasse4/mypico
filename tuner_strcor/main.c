/**
 * @name tuner_strcor
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This is an improvement over tuner_strings program:
 * the console display is performed asynchonously on core0 while
 * frequency detection is performed on core1.
 * data is interchanged between cores using a mutex mechanism.
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-04
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
// #include "pico/sync.h"

#include "fdetect.h"
#include "fedge.h"

// GPIO pin on which zero crossing are detected
#define INPUT_PIN       14

// number of period summed between falling edges to evaluate frequency
#define LENGTH          20
#if LENGTH > MAX_LENGTH
    #error LENGTH must be smaller or equal to MAX_LENGTH (fedge.h)
#endif

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA           0.01

// A non-linear filter (limit.c) is used to discard output frequencies 
// that are to far from the previously measured frequency.
// MAX_FREQ_PCT is that maximum percentage allowed from the previous measurment.
#define MAX_FREQ_PCT    0.05

// *************

typedef struct {
    double frequency;
    int status;
    uint32_t timestamp;
} tuner_t;

static mutex_t mutex;
static tuner_t shared_data;

void put_tuner_data(tuner_t *tuner)
{
    mutex_enter_blocking(&mutex);
    shared_data.frequency = tuner->frequency;
    shared_data.status = tuner->status;
    shared_data.timestamp = tuner->timestamp;
    mutex_exit(&mutex);    
}

void get_tuner_data(tuner_t *tuner) 
{
    mutex_enter_blocking(&mutex);
    tuner->frequency = shared_data.frequency;
    tuner->status = shared_data.status;
    tuner->timestamp = shared_data.timestamp;
    mutex_exit(&mutex);    
}

// *************



/*
 * Basse guitar string frequencies in Hz
 * B0   30.87
 * E1   41.21
 * A1   55.00
 * D2   74.42
 * G2   98.00
 * 
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110
 * D3   146.83
 * G3   196
 * B3   246.94
 * E4   329.63
 */

// frequency detection parameters
#define MAX_FREQ        410         // in Hz based on guitar high string
#define MIN_FREQ        25          // in Hz based on bass guitar low string
#define TIMER_FREQ      1000000     // in Hz
#define MAX_PERIOD_US  (LENGTH*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LENGTH*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS   801   // sleep time must be greater than the maximum period to be measured
#if (SLEEP_TIME_MS*1000) <= MAX_PERIOD_US
    #error Sleep time must be greater than the maximum period to be measured
#endif

#define BASS                0
#define GUITAR              1
#define NUM_STRING_GUITAR   6           // number of strings on a guitar
#define NUM_STRING_BASS     5           // number of strings on a bass
#define BASE_FREQUENCY      220.0       // base frequency (A3) used to calculate cents
#define LABEL_LEN           3

// strings cents relative to BASE_FREQ_GUITAR
double guitar_string_cents[NUM_STRING_GUITAR] = {
    -1700.0,    // E2
    -1200.0,    // A2
     -700.0,    // D3
     -200.0,    // G3
     +200.0,    // B3
     +700.0     // E4
};

char guitar_string_label[NUM_STRING_GUITAR][LABEL_LEN+1] = {
    "E2 ",
    "A2 ",
    "D3 ",
    "G3 ",
    "B3 ",
    "E4 "
};

// strings cents relative to BASE_FREQ_BASS
double bass_string_cents[NUM_STRING_BASS] = {
    -3400.0,    // B0
    -2900.0,    // E1
    -2400.0,    // A1
    -1900.0,    // D2
    -1400.0     // G2
};

char bass_string_label[NUM_STRING_BASS][LABEL_LEN+1] = {
    "B0 ",
    "E1 ",
    "A1 ",
    "D2 ",
    "G2 "
};

// Return the string that is the nearest the specified cents value
// Guitar: string 0 is the low E2, string 1 is A2, up to string 5 (E4)
// Bass:   string 0 is the low B0, string 2 is E1, up to string 4 (G2) 
int find_closest_string(int instrument, double cents)
{
    int i;
    double diff;
    int string;
    double min_diff = DBL_MAX;  // set to very large value
    double *string_cents;
    int n_strings;

    if (instrument == BASS) {
        n_strings = NUM_STRING_GUITAR;
        string_cents = bass_string_cents;
    }
    else {
        n_strings = NUM_STRING_BASS;
        string_cents = guitar_string_cents;
    }

    for (i = 0; i < n_strings; i++) {
        diff = fabs(cents - string_cents[i]);
        if (diff < min_diff) {
            min_diff = diff;
            string = i;
        }
    }
    return string;
}

// Compute cents value of a frequency relative to BASE_FREQUENCY
double get_cents(double frequency)
{
    return 1200.0 * logf(frequency/BASE_FREQUENCY) / M_LN2;
}

#define DISPLAY_LEN     101
#define BLANK           "                                                                                                    "
#define MID_POS         (DISPLAY_LEN/2)
#define DISP_FACTOR     ((DISPLAY_LEN-1)/500.0)

void show_frequency(double frequency)
{
    int i;
    char    s[1000];
    char    t[1000];
    double  cents = get_cents(frequency);
    int     string = find_closest_string(GUITAR, cents);
    int     rel_to_mid = round((cents - guitar_string_cents[string])*DISP_FACTOR);

    strcpy(s, BLANK);
    if (rel_to_mid < 0) {
        if (MID_POS + rel_to_mid < 0) return;
        for (i = MID_POS+rel_to_mid; i < MID_POS; i++) {
            s[i] = '*';
        }
    }
    else if (rel_to_mid > 0) {
        if (MID_POS + rel_to_mid > DISPLAY_LEN) return;
        for (i = MID_POS+1; i <= MID_POS+rel_to_mid; i++) {
            s[i] = '*';
        }
    }
    else {
        s[MID_POS] = '*';
    }

    s[MID_POS] = '|';
    s[DISPLAY_LEN+1] = '\0';
    strcpy(t, guitar_string_label[string]);
    strcat(t, s);
    printf("%s", t);
}



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

#define TOO_LOW     "  TOO LOW  "
#define TOO_HIGH    "  TOO HIGH "
#define NO_SIGNAL   " NO SIGNAL "
#define DETECTING   " DETECTING "

#define N_FREQ_OK        0
#define N_NO_SIGNAL     -1
#define N_DETECTING     -2
#define N_LOW_FREQ      -3
#define N_HIGH_FREQ     -4

void print_invalid(char *s)
{
    static char c[4] = {'|', '/', '-', '\\'};
    static int i = 0;
    printf("     %c  %s  %c  ", c[i], s, c[i]);
    printf("        \r");
    i = (i +1) % 4;
}

// Program entry point
void core1_entry() {
    tuner_t tuner;
    double frequency;
    bool status;

    while (true) {
        
        // check if timer has elapsed
        // its either because there is no signal (no falling edge)
        // or the signal frequency is too low to get enough counts
        if (fdetect_is_timer_elapsed()) {
            if (fdetect_no_falling_edge()) {
                tuner.status = N_NO_SIGNAL;
            }
            else {
                tuner.status = N_LOW_FREQ;
            }

            tuner.frequency = 0.0;
            tuner.timestamp = time_us_32();
            put_tuner_data(&tuner);
            fdetect_reset_search();
            toggle_led();
        }

        // check if the falling edge counter has successfully performed
        else if (fdetect_is_read_ok()) {

            status = fdetect_get_frequency(&frequency);
            // if true is returned, the frequency is good (maybe too high)
            if (status) {
                if (frequency > MAX_FREQ) {
                    tuner.frequency = frequency;
                    tuner.status = N_HIGH_FREQ;
                }
                else {
                    tuner.frequency = frequency;
                    tuner.status = N_FREQ_OK;
                }
            }
            // a false means that the frequency has rapidely changed and should be discarded
            else {
                tuner.frequency = 0.0;
                tuner.status = N_DETECTING;
            }
            tuner.timestamp = time_us_32();
            put_tuner_data(&tuner);
            toggle_led();            
        }
    }
}


// Program entry point
int main() {
    // double frequency;
    // int status;
    tuner_t tuner;
    uint32_t start_time = 0;
    uint32_t elapsed_time, now;
    uint32_t timestamp = 0;

    // initialize printing to console
    stdio_init_all();

    // Inititialize on board LED (live signal)
    init_led();   
    
    // Set a repeating alarm that is called at each SLEEP_TIME_US
    sleep_ms(100);  // it seems that the internal timer takes some time to startup.
    fdetect_init(INPUT_PIN, LENGTH, SLEEP_TIME_MS, ALPHA, MAX_FREQ_PCT);

    mutex_init(&mutex);
    multicore_launch_core1(core1_entry);

    print_invalid(DETECTING);
    printf("        \r");

    while (true) {

        sleep_ms(5);
        get_tuner_data(&tuner);

        if (tuner.timestamp == timestamp) continue;
        timestamp = tuner.timestamp;

        now = time_us_32();
        elapsed_time =  now - start_time;
        start_time = now;

        switch (tuner.status) {
        case N_FREQ_OK:
            printf("  %7.3lf", tuner.frequency);
            printf("  %lu   ", elapsed_time);
            printf("        \r");
            break;

        case N_NO_SIGNAL:
            print_invalid(NO_SIGNAL);   
            break;

        case N_LOW_FREQ:
            print_invalid(TOO_LOW);
            break;

        case N_HIGH_FREQ:
            print_invalid(TOO_HIGH);
            break;

        case N_DETECTING:
            print_invalid(DETECTING);
            break;

        default:
            printf("should never occur\n");
            break;
        }
    }

    fdetect_release();
}
