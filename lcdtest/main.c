/** Sample test for Waveshare 1.14 LCD display for Pico 
 * 
 */

#include <math.h>
#include <float.h>
#include <string.h>
// #include "EPD_Test.h"
#include "LCD_1in14.h"
#include "GUI_Paint.h"
#include "ImageData.h"

/* set address */
// bool reserved_addr(uint8_t addr) {
// return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
// }

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
#define TUNER_HEIGHT    50
#define T_HALF_LENGTH   (TUNER_LENGTH/2)
#define CENTS_SHOWN     200.0
#define DISP_FACTOR     ((TUNER_LENGTH-1)/CENTS_SHOWN)
#define WX0 (MID_POS-T_HALF_LENGTH)
#define WX1 (MID_POS+T_HALF_LENGTH)
#define WY0 10
#define WY1 (WX0+TUNER_HEIGHT)
#define CENTS_LIMIT 10.0

#define YPOS_DIRECT     65
#define XGAP_DIRECT     10
#define YPOS_MESSAGE    80
#define YPOS_SIGN       110
#define LOW_SIGN        "<<<<"
#define HIGH_SIGN       ">>>>"
#define LOW_STR         "bas"
#define HIGH_STR        "haut"
#define TOO_LOW_STR     "TROP BAS"
#define TOO_HIGH_STR    "TROP HAUT"
#define NO_SIGNAL       "SIGNAL ABSENT"

void display_tuner(double frequency)
{
    int i;
    
    uint16_t x0, x1;
    uint16_t color;
    bool    is_centered;
    bool    is_out_of_tune;
    bool    show_low_sign;
    bool    show_high_sign;
    int     pos;
    int     string;
    double  cents;
    double  cents_delta; 

    Paint_Clear(BLACK);
    Paint_DrawRectangle(WX0, WY0, WX1, WY1, GRAY, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    Paint_DrawLine(MID_POS, WY0-5, MID_POS, WY1+5, WHITE, DOT_PIXEL_1X1, LINE_STYLE_DOTTED);
    Paint_DrawString_EN(XGAP_DIRECT, YPOS_DIRECT, LOW_STR, &Font12, BLACK, WHITE);
    Paint_DrawString_EN(LCD_1IN14_HEIGHT - XGAP_DIRECT - (7*strlen(HIGH_STR)) - XGAP_DIRECT, YPOS_DIRECT, HIGH_STR, &Font12, BLACK, WHITE);

    if (frequency <= 0.0) {
        Paint_DrawString_EN(MID_POS-(17*strlen(NO_SIGNAL)/2), YPOS_MESSAGE, NO_SIGNAL, &Font24, BLACK, WHITE);
    } 
    // else if (frequency < MIN_FREQ && frequency > 0.0) {
    //     Paint_DrawString_EN(MID_POS-(17*strlen(TOO_LOW_STR)/2), YPOS_MESSAGE, TOO_LOW_STR, &Font24, BLACK, WHITE);
    // }
    // else if (frequency > MAX_FREQ) {
    //     Paint_DrawString_EN(MID_POS-(17*strlen(TOO_HIGH_STR)/2), YPOS_MESSAGE, TOO_HIGH_STR, &Font24, BLACK, WHITE);
    // }
    else {
        cents = get_cents(frequency);
        string = find_closest_string(cents);
        cents_delta = cents - string_cents[string]; 
        pos = round(cents_delta)*DISP_FACTOR;
    
        is_centered = false;
        show_low_sign = false;
        show_high_sign = false;
        is_out_of_tune = fabs(cents_delta) >= CENTS_LIMIT;
    
        color = RED;
        if (!is_out_of_tune) color = GREEN; 
    
        if (pos < 0) {
            x1 = MID_POS - 1;
            if (MID_POS + pos > WX0) {
                x0 = MID_POS + pos;
            }
            else {
                x0 = WX0 + 1;
                show_low_sign = true;
            }
        }
        else if (pos > 0) {
            x0 = MID_POS + 1;
            if (MID_POS + pos < WX1) {
                x1 = MID_POS + pos;
            }
            else {
                x1 = WX1 - 1;
                show_high_sign = true;
            }
        }
        else {
            is_centered = true;
        }
    
        if (is_centered) {
            Paint_DrawLine(MID_POS, WY0-5, MID_POS, WY1, GREEN, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        }
        else {
            Paint_DrawRectangle(x0, WY0+1, x1, WY1-1, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
        if (show_low_sign) {
            Paint_DrawString_EN(MID_POS-(T_HALF_LENGTH/2)-(17*strlen(LOW_SIGN)/2), YPOS_SIGN, LOW_SIGN, &Font24, BLACK, RED);
        }
        if (show_high_sign) {
            Paint_DrawString_EN(MID_POS+(T_HALF_LENGTH/2)-(17*strlen(HIGH_SIGN)/2), YPOS_SIGN, HIGH_SIGN, &Font24, BLACK, RED);
        }
        Paint_DrawString_EN(MID_POS-(17*strlen(string_label[string])/2), YPOS_MESSAGE, string_label[string], &Font24, BLACK, WHITE);
    }

    LCD_1IN14_Display(image);
}

#define DELAY_MS 5

int main(void)
{
    LCD_init();

    while (true) {

        for (int i = -10; i < 410; i++) {
            display_tuner(i);
            DEV_Delay_ms(DELAY_MS);
        }
    }
    
    DEV_Module_Exit();

    return 0;
}
