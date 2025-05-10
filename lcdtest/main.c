/** Sample test for Waveshare 1.14 LCD display for Pico 
 * 
 */

#include <math.h>
#include <float.h>
#include <string.h>
#include "LCD_1in14.h"
#include "GUI_Paint.h"

#define IMAGE_SIZE (LCD_1IN14_HEIGHT*LCD_1IN14_WIDTH*2)
UWORD image[IMAGE_SIZE];

void LCD_init() 
{
    DEV_Delay_ms(100);
    DEV_Module_Init();
    DEV_SET_PWM(50);    // back light level (0 - 100)

    /* LCD Init */
    LCD_1IN14_Init(HORIZONTAL);
    LCD_1IN14_Clear(WHITE);
    
    // Create a new image cache and fill it with white*/
    Paint_NewImage((UBYTE *)image, LCD_1IN14.WIDTH, LCD_1IN14.HEIGHT, 0, BLACK);
    Paint_SetScale(65);
    Paint_Clear(WHITE);
    Paint_SetRotate(ROTATE_0);

    // flash
    Paint_Clear(WHITE);
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

/*
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110
 * D3   146.83
 * G3   196
 * B3   246.94
 * E4   329.63
 */

#define NUM_STRING      6           // number of strings on a guitar
#define BASE_FREQUENCY  220.0       // base frequency (A3) used to calculate cents
#define LABEL_LEN       2

// strings cents relative to BASE_FREQUENCY
double string_cents[NUM_STRING] = {
    -1700.0,    // E2
    -1200.0,    // A2
     -700.0,    // D3
     -200.0,    // G3
     +200.0,    // B3
     +700.0     // E4
};

char *string_label[NUM_STRING]= {
    "Mi Bas",
    "La",
    "Re",
    "Sol",
    "Si",
    "Mi Aigu"
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
    return 1200.0 * logf(frequency/BASE_FREQUENCY) / _M_LN2;
}

#define MAX_FREQ        400         // in Hz
#define MIN_FREQ        75          // in Hz

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

#define YPOS_DIRECT     (WY1+5)
#define XGAP_DIRECT     10
#define YPOS_NOTE       (WY1+30)
#define LOW_DIRECT_STR  "bas"
#define HIGH_DIRECT_STR "haut"

#define YPOS_SIGN       (WY1+30+30)
#define LOW_SIGN_STR    "<<<<"
#define HIGH_SIGN_STR   ">>>>"
#define NO_SIGN_STR     "    "
#define NO_SIGN         0
#define HIGH_SIGN       1
#define LOW_SIGN        2

#define N_FREQ_OK       0
#define N_NO_SIGNAL     -1
#define N_DETECTING     -2
#define N_LOW_FREQ      -3
#define N_HIGH_FREQ     -4

#define NO_SIGNAL_STR   "SIGNAL ABSENT"
#define DETECTING_STR   "DETECTION..."
#define LOW_FREQ_STR    "TROP BAS"
#define HIGH_FREQ_STR   "TROP AIGU"

// #define NO_SIGNAL       false
// #define SIGNAL_DETECTED true

// bool full_redraw = true;
bool tuner_shown = false;
bool low_sign_shown = false;
bool high_sign_shown = false;

uint16_t last_x0 = 0;
uint16_t last_x1 = 0;
int last_note = -1;
int last_note_strlen;

void draw_string_window(uint16_t x0, uint16_t y0, const char *s, sFONT *f, uint16_t fore, uint16_t back)
{
    Paint_DrawString_EN(x0, y0, s, f, fore, back);
    LCD_1IN14_DisplayWindows(x0, y0, x0 + f->Width*strlen(s), y0 + f->Height, image);
}

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

void erase_tuner()
{
    if (tuner_shown) {
        Paint_DrawRectangle(WX0+1, WY0+1, WX1-1, WY1-1, BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        LCD_1IN14_DisplayWindows(WX0+1, WY0+1, WX1-1, WY1-1, image);
        tuner_shown = false;
    }
}

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
            default:          msg = string_label[note]; break;
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
        cents_delta = cents - string_cents[string]; 
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

#define DELAY_MS 50

int main(void)
{
    uint32_t start;
    int status;

    stdio_init_all();
    sleep_ms(1000);
    printf("lcdtest\n");


    LCD_init();

    display_bckgnd();
    while (true) {

        for (double i = -10.0; i < 410.0; i=i+0.5) {

            status = 0;
            if (i < 0.0) status = N_NO_SIGNAL;
            else if (i < 25.0) status = N_DETECTING; 
            else if (i < 60.0) status = N_LOW_FREQ;
            else if (i > 400.0) status = N_HIGH_FREQ;

            start = time_us_32();
            display_tuner(status, i);
            printf("%8lu\n", time_us_32() - start);
            DEV_Delay_ms(DELAY_MS);

        }
    }
    
    DEV_Module_Exit();

    return 0;
}
