/**
 * @name tuner_lcd
 * @details
 * Simple bass/guitar tuner using an zero crossing algorithm
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 14 (INPUT_PIN) for frequency estimation 
 *  - ouptuts to console 
 * 
 * This is an improvement over tuner_strcor program:
 * the console display has been replaced by a WaveShare 1.14 lcd 
 * (https://www.waveshare.com/wiki/Pico-LCD-1.14)
 * This a merge of tuner_core and lcdtest projects
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-12
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include <assert.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
// #include "pico/sync.h"

// local modules
#include "fdetect.h"
#include "fedge.h"

// LCD modules
#include "LCD_1in14.h"
#include "GUI_Paint.h"

// GPIO pin on which zero crossing are detected
#define INPUT_PIN       4

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

/*
 * Data structure and functions to pass tuner information from 
 * core1 (frequency measurement) to core2 (display management)
 */

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


/*
 * Instrument specific data and functions
 *
 * Bass guitar string frequencies in Hz
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

char *guitar_string_label[NUM_STRING_GUITAR] = {
    "Mi Bas",
    "La",
    "Re",
    "Sol",
    "Si",
    "Mi Aigu"
};

// strings cents relative to BASE_FREQ_BASS
double bass_string_cents[NUM_STRING_BASS] = {
    -3400.0,    // B0
    -2900.0,    // E1
    -2400.0,    // A1
    -1900.0,    // D2
    -1400.0     // G2
};

char *bass_string_label[NUM_STRING_BASS] = {
    "Si",
    "Mi",
    "La",
    "Re",
    "Sol"
};

static int instrument = GUITAR;
void set_instrument(int _instrument)    { instrument = _instrument; }
int  get_instrument()                   { return instrument; }
char* get_string_label(int string) 
{ 
    if (instrument == GUITAR) {
        return guitar_string_label[string];
    }
    else {
        return bass_string_label[string];
    }
}

double get_string_cents(int string) {
    if (instrument == GUITAR) {
        return guitar_string_cents[string];
    }
    else {
        return bass_string_cents[string];
    }
}

// Return the string that is the nearest the specified cents value
// Guitar: string 0 is the low E2, string 1 is A2, up to string 5 (E4)
// Bass:   string 0 is the low B0, string 2 is E1, up to string 4 (G2) 
int find_closest_string(double cents)
{
    int i;
    double diff;
    int string;
    double min_diff = DBL_MAX;  // set to very large value
    double *string_cents;
    int n_strings;

    if (instrument == GUITAR) {
        n_strings = NUM_STRING_GUITAR;
        string_cents = guitar_string_cents;
    }
    else {
        n_strings = NUM_STRING_BASS;
        string_cents = bass_string_cents;
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

/*
 * Waveshare 1.14 LCD functions
 *
 * These functions were designed to minimized data transmission
 * over the spi link which proved to be lenghty using the library 
 * provided by Waveshare.
 * 
 * The display is divided in 4 regions: tuner, sign, note and bacground.
 * The background is the entire screen, and should be drawn once at the beginning.
 * The tuner region is a bargraph showing the frequency relative to a tuning note.
 * The sign region shows low or high signs if the frequency exceeds the bargraph capability.
 * the note region shows the tuning note or some messages related to the tuner status.
 * 
 * Note: LCD_1IN14_Clear() should NOT be used since it stalls the processor.  
 * I suspect its automatic variables are to big.
 */

// definitions to setup the tuner display

//  tuner position
#define MID_POS         (LCD_1IN14_HEIGHT/2)
#define TUNER_LENGTH    221
#define TUNER_HEIGHT    10
#define T_HALF_LENGTH   (TUNER_LENGTH/2)
#define CENTS_SHOWN     200.0
#define DISP_FACTOR     ((TUNER_LENGTH-1)/CENTS_SHOWN)
#define WX0             (MID_POS-T_HALF_LENGTH)
#define WX1             (MID_POS+T_HALF_LENGTH)
#define WY0             20
#define WY1             (WY0+TUNER_HEIGHT)
#define TIC_LEN         5
#define CENTS_LIMIT     20.0

// small direction labels
#define YPOS_DIRECT     (WY1+5)
#define XGAP_DIRECT     10
#define YPOS_NOTE       (WY1+30)
#define LOW_DIRECT_STR  "bas"
#define HIGH_DIRECT_STR "haut"

// low and hign signs
#define YPOS_SIGN       (WY1+30+30)
#define LOW_SIGN_STR    "<<<<"
#define HIGH_SIGN_STR   ">>>>"
#define NO_SIGN_STR     "    "
#define NO_SIGN         0
#define HIGH_SIGN       1
#define LOW_SIGN        2

// note message labels
#define NO_SIGNAL_STR   "SIGNAL ABSENT"
#define DETECTING_STR   "DETECTION..."
#define LOW_FREQ_STR    "TROP BAS"
#define HIGH_FREQ_STR   "TROP AIGU"

// frequency status definitions
#define N_FREQ_OK        0
#define N_NO_SIGNAL     -1
#define N_DETECTING     -2
#define N_LOW_FREQ      -3
#define N_HIGH_FREQ     -4


#define IMAGE_SIZE (LCD_1IN14_HEIGHT*LCD_1IN14_WIDTH*2)
UWORD image[IMAGE_SIZE];

bool tuner_shown = false;
bool low_sign_shown = false;
bool high_sign_shown = false;

uint16_t last_x0 = 0;
uint16_t last_x1 = 0;
int last_note = -1;
int last_note_strlen;

// Waveshare LCD initialisation.
void LCD_init() 
{
    DEV_Delay_ms(100);
    DEV_Module_Init();
    DEV_SET_PWM(50);    // back light level (0 - 100)

    /* LCD Init */
    LCD_1IN14_Init(HORIZONTAL);
    // LCD_1IN14_Clear(WHITE);

    // Create a new image cache and fill it with white*/
    Paint_NewImage((UBYTE *)image, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, 0, BLACK);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);

    // flash
    LCD_1IN14_Display(image);
    Paint_Clear(BLACK);
    LCD_1IN14_Display(image);
    Paint_Clear(WHITE);
    LCD_1IN14_Display(image);
    Paint_Clear(BLACK);
    LCD_1IN14_Display(image);
    Paint_Clear(WHITE);
    LCD_1IN14_Display(image);
}

// draw a string immediate at position
// note: fore and back are reversed
void draw_string_window(uint16_t x0, uint16_t y0, const char *s, sFONT *f, uint16_t fore, uint16_t back)
{
    Paint_DrawString_EN(x0, y0, s, f, fore, back);
    LCD_1IN14_DisplayWindows(x0, y0, x0 + f->Width*strlen(s), y0 + f->Height, image);
}

// draw the full display background
void display_bckgnd()
{
    size_t len = strlen(NO_SIGNAL_STR);
    Paint_Clear(BLACK);
    Paint_DrawRectangle(WX0, WY0, WX1, WY1, GRAY, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawLine(MID_POS, WY0-TIC_LEN, MID_POS, WY0, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(MID_POS, WY1, MID_POS, WY1+TIC_LEN, WHITE, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawString_EN(XGAP_DIRECT, YPOS_DIRECT, LOW_DIRECT_STR, &Font12, BLACK, WHITE);
    Paint_DrawString_EN(LCD_1IN14_HEIGHT - XGAP_DIRECT - (Font12.Width*strlen(HIGH_DIRECT_STR)), YPOS_DIRECT, HIGH_DIRECT_STR, &Font12, BLACK, WHITE);
    Paint_DrawString_EN(MID_POS-(Font24.Width*len/2), YPOS_NOTE, NO_SIGNAL_STR, &Font24, BLACK, WHITE);

    LCD_1IN14_Display(image);

    tuner_shown = false;
    low_sign_shown = false;
    high_sign_shown = false;
    last_x0 = 0;
    last_x1 = 0;
    last_note_strlen = len;
    last_note = N_NO_SIGNAL;
}

// erase the tuner region, if necessary
void erase_tuner()
{
    if (tuner_shown) {
        Paint_DrawRectangle(WX0+1, WY0+1, WX1-1, WY1-1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        LCD_1IN14_DisplayWindows(WX0+1, WY0+1, WX1-1, WY1-1, image);
        tuner_shown = false;
    }
}

// draw the tuner region, if necessary
void show_tuner(uint16_t x0, uint16_t x1, uint16_t color)
{
    if (!tuner_shown || x0 != last_x0 || x1 != last_x1) {
        Paint_DrawRectangle(WX0+1, WY0+1, WX1-1, WY1-1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        Paint_DrawRectangle(x0, WY0+1, x1, WY1-1, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        LCD_1IN14_DisplayWindows(WX0+1, WY0+1, WX1-1, WY1-1, image);
        tuner_shown = true;
        last_x0 = x0;
        last_x1 = x1;
    }
}

// erase the low and high signs, if necessary
void erase_sign() 
{
    uint16_t x;

    if (low_sign_shown) {
        x = MID_POS-(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
        draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        low_sign_shown = false;
    }
    if (high_sign_shown) {
        x = MID_POS+(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
        draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        high_sign_shown = false;
    }
}

// draw the low or the high sign, if necessary
void show_sign(int sign)
{
    uint16_t x;

    if (sign == LOW_SIGN) {
        if (high_sign_shown) {
            // erase high sign
            x = MID_POS+(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        }
        if (!low_sign_shown) {
            // draw low sign
            x = MID_POS-(T_HALF_LENGTH/2)-(Font24.Width*strlen(LOW_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, LOW_SIGN_STR, &Font24, BLACK, RED);
        }
        high_sign_shown = false;
        low_sign_shown = true;
    }
    else if (sign == HIGH_SIGN) {
        if (low_sign_shown) {
            // erase low sign
            x = MID_POS-(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        }
        if (!high_sign_shown) {
            // draw high sign
            x = MID_POS+(T_HALF_LENGTH/2)-(Font24.Width*strlen(HIGH_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, HIGH_SIGN_STR, &Font24, BLACK, RED);
        }
        high_sign_shown = true;
        low_sign_shown = false;
    }
    else if (sign == NO_SIGN) {
        // erase both signs
        if (low_sign_shown) {
            x = MID_POS-(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        }
        if (high_sign_shown) {
            x = MID_POS+(T_HALF_LENGTH/2)-(Font24.Width*strlen(NO_SIGN_STR)/2);
            draw_string_window(x, YPOS_SIGN, NO_SIGN_STR, &Font24, BLACK, BLACK);
        }
        high_sign_shown = false;
        low_sign_shown = false;
    }
}

// erase the note region, if necessary
void erase_note() 
{
    uint16_t x0, x1, y0, y1;
    size_t len;

    if (last_note != N_NO_SIGNAL) {
        x0 = MID_POS - (Font24.Width*last_note_strlen/2);
        x1 = x0 + last_note_strlen*Font24.Width;
        y0 = YPOS_NOTE;
        y1 = YPOS_NOTE + Font24.Height;
        len = strlen(NO_SIGNAL_STR);

        Paint_DrawRectangle(x0, y0, x1, y1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);  // erase
        Paint_DrawString_EN(MID_POS-(Font24.Width*len/2), YPOS_NOTE, NO_SIGNAL_STR, &Font24, BLACK, WHITE);
        LCD_1IN14_DisplayWindows(x0, YPOS_NOTE, x1, y1, image);  // send do display

        last_note_strlen = len;
        last_note = N_NO_SIGNAL;
    }
}

// draw the note, if necessary
void show_note(int note) 
{
    uint16_t x0, x1, y0, y1, dx0, dx1;
    size_t len;
    char *msg;

    if (last_note != note) {
        x0 = MID_POS - (Font24.Width*last_note_strlen/2);
        x1 = x0 + last_note_strlen*Font24.Width;
        y0 = YPOS_NOTE;
        y1 = YPOS_NOTE + Font24.Height;

        switch (note) {
            case N_NO_SIGNAL: msg = NO_SIGNAL_STR;      break;
            case N_DETECTING: msg = DETECTING_STR;      break;
            case N_LOW_FREQ:  msg = LOW_FREQ_STR;       break;
            case N_HIGH_FREQ: msg = HIGH_FREQ_STR;      break;
            default:          msg = get_string_label(note); break;
        }

        len = strlen(msg);
        dx0 = MID_POS-(Font24.Width*len/2);
        dx1 = dx0 + len*Font24.Width;

        Paint_DrawRectangle(x0, y0, x1, y1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);  // erase
        Paint_DrawString_EN(dx0, y0, msg, &Font24, BLACK, WHITE);  // draw
        LCD_1IN14_DisplayWindows((x0 < dx0 ? x0 : dx0), YPOS_NOTE, (x1 > dx1 ? x1 : dx1), y1, image);  // send do display

        last_note_strlen = len;
        last_note = note;
    }
}

// display the tuner region, if necessary
void display_tuner(int status, double frequency)
{    
    uint16_t x0, x1;
    uint16_t color;
    bool    is_out_of_tune;
    bool    show_low_sign;
    bool    show_high_sign;
    int     pos;
    int     string;
    double  cents;
    double  cents_delta; 

    switch (status) {
    case N_NO_SIGNAL:
        erase_tuner();
        erase_sign();
        show_note(N_NO_SIGNAL);
        break;

    case N_DETECTING:
        erase_tuner();
        erase_sign();
        show_note(N_DETECTING);
        break;

    case N_LOW_FREQ:
        erase_tuner();
        show_sign(LOW_SIGN);
        show_note(N_LOW_FREQ);
        break;

    case N_HIGH_FREQ:
        erase_tuner();
        show_sign(HIGH_SIGN);
        show_note(N_HIGH_FREQ);
        break;

    case N_FREQ_OK:
        cents = get_cents(frequency);
        string = find_closest_string(cents);
        cents_delta = cents - get_string_cents(string); 
        pos = round(cents_delta)*DISP_FACTOR;
    
        show_low_sign = false;
        show_high_sign = false;
        is_out_of_tune = fabs(cents_delta) >= CENTS_LIMIT;
    
        color = RED;
        if (!is_out_of_tune) color = GREEN; 
    
        if (pos < 0) {
            // flat - draw to the left
            x1 = MID_POS;
            if (MID_POS + pos > WX0) {
                x0 = MID_POS + pos;
            }
            else {
                x0 = WX0 + 1;
                show_low_sign = true;
            }
        }
        else if (pos > 0) {
            // sharp - draw on the right
            x0 = MID_POS;
            if (MID_POS + pos < WX1) {
                x1 = MID_POS + pos;
            }
            else {
                x1 = WX1 - 1;
                show_high_sign = true;
            }
        }
        else {      
            // tuned - draw centered
            x0 = MID_POS;
            x1 = MID_POS;
        }
    
        show_tuner(x0, x1, color);

        if (show_low_sign) {
            show_sign(LOW_SIGN);
        }
        else if (show_high_sign) {
            show_sign(HIGH_SIGN);
        }
        else {
            show_sign(NO_SIGN);
        }
        show_note(string);
        break;
    }
}

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

void print_invalid(char *s)
{
    static char c[4] = {'|', '/', '-', '\\'};
    static int i = 0;
    printf("     %c  %s  %c  ", c[i], s, c[i]);
    printf("        \r");
    i = (i +1) % 4;
}


/* 
 * Entry point for core1 processing
 * 
 * core1 listens to INPUT_PIN GPIO and measures the signal frequency.
 * It uses a GPIO interrupt and a timer interrupt.
 */ 

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

/*
 * Entry point for core0 processing
 *
 * core0 initiates and launches core1. It is also responsible for displaying the tuner.
 */

#define MSG_TOO_LOW     "  TOO LOW  "
#define MSG_TOO_HIGH    "  TOO HIGH "
#define MSG_NO_SIGNAL   " NO SIGNAL "
#define MSG_DETECTING   " DETECTING "

// frequency status definitions
// #define N_FREQ_OK        0
// #define N_NO_SIGNAL     -1
// #define N_DETECTING     -2
// #define N_LOW_FREQ      -3
// #define N_HIGH_FREQ     -4


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

    // setup mutex for data interchanges between core1 and core0
    mutex_init(&mutex);

    // launch frequency measurment
    multicore_launch_core1(core1_entry);

    // Initialise the LCD and display the tuner background.
    // LCD_init();
    // display_bckgnd();

    while (true) {

        sleep_ms(5);
        get_tuner_data(&tuner);

        // check that the timestamp has changed in order to 
        // modify the display only if data has changed.
        if (tuner.timestamp == timestamp) continue;
        timestamp = tuner.timestamp;

        // to be removed
        now = time_us_32();
        elapsed_time =  now - start_time;
        start_time = now;

        printf("  f: %7.3lf  s: %d\n", tuner.frequency, tuner.status);
        continue;

        switch (tuner.status) {
        case N_FREQ_OK:
            display_tuner(tuner.status, tuner.frequency);

            // printf("  %7.3lf", tuner.frequency);
            // printf("  %8lu   ", elapsed_time);
            // printf("        \r");
            break;

        case N_NO_SIGNAL:
            display_tuner(tuner.status, tuner.frequency);
            print_invalid(MSG_NO_SIGNAL);   
            break;

        case N_LOW_FREQ:
            display_tuner(tuner.status, tuner.frequency);
            print_invalid(MSG_TOO_LOW);
            break;

        case N_HIGH_FREQ:
            display_tuner(tuner.status, tuner.frequency);
            printf("  %7.3lf", tuner.frequency);
            print_invalid(MSG_TOO_HIGH);
            break;

        case N_DETECTING:
            display_tuner(tuner.status, tuner.frequency);
            print_invalid(MSG_DETECTING);
            break;

        default:
            printf("should never occur\n");
            break;
        }
    }

    fdetect_release();
}
