/**
 * @name tuner_string
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This is an improvement over tuner_cnt10_nw program:
 * the console display shows the frequency graphically and
 * the string being tuned is shown.
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

#include "fdetect.h"

// GPIO pin on which zero crossing are detected
#define INPUT_PIN       14

// A one tap IIR filter is applied to the output frequency (alpha.c)
// ALPHA is the IIR filter weight
#define ALPHA           0.01

// A non-linear filter (limit.c) is used to discard output frequencies 
// that are to far from the previously measured frequency.
// MAX_FREQ_PCT is that maximum percentage allowed from the previous measurment.
#define MAX_FREQ_PCT    0.10

// frequency detection parameters
#define MAX_FREQ        400         // in Hz
#define MIN_FREQ        75          // in Hz
#define TIMER_FREQ      1000000     // in Hz
#define MAX_PERIOD_US  (LENGTH*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LENGTH*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS   300   // must be physically greater than MAX_PERIOD_US
#if (SLEEP_TIME_MS*1000) <= MAX_PERIOD_US
    #error SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif

#define NUM_STRING      6           // number of strings on a guitar
#define BASE_FREQUENCY  220.0       // base frequency (A3) used to calculate cents
#define LABEL_LEN       3

/*
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110
 * D3   146.83
 * G3   196
 * B3   246.94
 * E4   329.63
 */

// strings cents relative to BASE_FREQUENCY
double string_cents[NUM_STRING] = {
    -1700.0,    // E2
    -1200.0,    // A2
     -700.0,    // D3
     -200.0,    // G3
     +200.0,    // B3
     +700.0     // E4
};

char string_label[NUM_STRING][LABEL_LEN+1] = {
    "E2 ",
    "A2 ",
    "D3 ",
    "G3 ",
    "B3 ",
    "E4 "
};


// Return the string that is the nearest the specified cents value
// String 0 is the low E, string 1 is A, on so on to string 5 (high E)
int find_closest_string(double cents)
{
    int i;
    double diff;
    int string;
    double min_diff = DBL_MAX;  // set to very large value

    for (i = 0; i < 6; i++) {
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
    int     string = find_closest_string(cents);
    int     rel_to_mid = round((cents - string_cents[string])*DISP_FACTOR);

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
    strcpy(t, string_label[string]);
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
void print_invalid(char *s)
{
    static char c[4] = {'|', '/', '-', '\\'};
    static int i = 0;
    printf("     %c  %s  %c %60s", c[i], s, c[i], " ");
    i = (i +1) % 4;
}

// Program entry point
int main() {
    double frequency;

    // initialize printing to console
    stdio_init_all();

    // Inititialize on board LED (live signal)
    init_led();   
    
    // Set a repeating alarm that is called at each SLEEP_TIME_US
    sleep_ms(100);  // it seems that the internal timer takes some time to startup.
    fdetect_init(INPUT_PIN, SLEEP_TIME_MS, ALPHA, MAX_FREQ_PCT);

    while (true) {
        
        if (fdetect_is_ready()) {

            toggle_led();

            if (fdetect_get_frequency(&frequency)) {
                if (frequency < MIN_FREQ) {
                    print_invalid(TOO_LOW);
                }
                else if (frequency > MAX_FREQ) {
                    print_invalid(TOO_HIGH);
                }
                else {
                    show_frequency(frequency);
                    printf("  %6.2lf", frequency);
                }
            }
            else {
                print_invalid(NO_SIGNAL);
            }
            printf("\r");
        }
    }

    fdetect_release();
}
