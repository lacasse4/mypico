/**
 * freq_to_display.c
 *
 * Measure frequency on GPIO PIN_IRQ.
 * Display frequency arround 200Hz on a bar display driven by a MCP23018.
 */

/*
 * Guitar string frequencies in Hz
 * E2   82.41
 * A2   110
 * D3   146.83
 * G3   196
 * B3   246.94
 * E4   329.63
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// Define MCP23018 registers 
#define I2C_ADDRESS 0x20

#define IODIRA   0x00  // IO direction A - 1= input 0 = output
#define IODIRB   0x01  // IO direction B - 1= input 0 = output    
#define IPOLA    0x02  // Input polarity A
#define IPOLB    0x03  // Input polarity B
#define GPINTENA 0x04  // Interrupt-onchange A
#define GPINTENB 0x05  // Interrupt-onchange B
#define DEFVALA  0x06  // Default value for port A
#define DEFVALB  0x07  // Default value for port B
#define INTCONA  0x08  // Interrupt control register for port A
#define INTCONB  0x09  // Interrupt control register for port B
#define IOCON    0x0A  // Configuration register
#define GPPUA    0x0C  // Pull-up resistors for port A
#define GPPUB    0x0D  // Pull-up resistors for port B
#define INTFA    0x0E  // Interrupt condition for port A
#define INTFB    0x0F  // Interrupt condition for port B
#define INTCAPA  0x10  // Interrupt capture for port A
#define INTCAPB  0x11  // Interrupt capture for port B
#define GPIOA    0x12  // Data port A
#define GPIOB    0x13  // Data port B
#define OLATA    0x14  // Output latches A
#define OLATB    0x15  // Output latches B


#define LEN 10  // number of falling edges counted to evaluate PIN_IRQ frequency

#define MAX_FREQ    400         // in Hz
#define MIN_FREQ    75          // in Hz
#define TIMER_FREQ  1000000     // in Hz
#define MAX_PERIOD_US  (LEN*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LEN*TIMER_FREQ/MAX_FREQ)

#define SLEEP_TIME_MS 200  // must be physicaly greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be greater than MAX_PERIOD_US
#endif

const uint LED_PIN = PICO_DEFAULT_LED_PIN;
const uint PIN_IRQ = 14;
static bool led_on = true;

static volatile bool first = false;
static volatile bool done  = true;
static uint32_t start;
static uint32_t period;
static int counter;

void gpio_callback(uint gpio, uint32_t events) 
{
    uint32_t value = time_us_32();

    if (first) {
        start = value;
        first = false;
        return;
    }

    if (!done) {
        counter++;
        if (counter == LEN) {
            period = value - start;
            done = true;
        }
    }
}

void init_irq() 
{
    gpio_set_input_enabled (PIN_IRQ, true);
    gpio_pull_up (PIN_IRQ);
    gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}

void start_search_safe() 
{
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
    done  = false;
    first = true;
    period = 0;
    counter = 0;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);
}

void init_led() 
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

void toggle_led() 
{
    if (led_on) gpio_put(LED_PIN, 0);
    else        gpio_put(LED_PIN, 1);
    led_on = !led_on;
}

void init_i2c() 
{
    int ret;
    uint8_t opcode;
    uint8_t data[2];

    // use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // set MCP23018 IOCON register
    //   - set sequencial mode (bit 5 to 0)
    //   - set default interrupt pin level to LOW (bit 2 to 1) 
    opcode = IOCON;
    data[0] = 0x02;
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
    // printf("write iocon opcode ret= %d\n", ret);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 1, false);
    // printf("write iocon data ret= %d\n", ret);

    // set MCP23018 port A and B as outputs 
    opcode = IODIRA;
    data[0] = 0x00;
    data[1] = 0x00;
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
    // printf("write iodira opcode ret= %d\n", ret);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 2, false);
    // printf("write iodira data ret= %d\n", ret);
}

void refresh_i2c_display(uint16_t value) 
{
    int ret;
    uint8_t opcode;
    uint8_t data[2];
    value = ~value;  // a bit value of 0 lights the led.
    opcode = GPIOA;
    data[0] = (uint8_t)(value & 0x00FF);
    data[1] = (uint8_t)((value >> 8) & 0x00FF);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 2, false);
}

uint16_t frequency_to_display(double frequency) 
{
    int shift = (int)round(frequency-200.0) + 7;
    // printf("shift = %d", shift);
    if (shift < 0) return 0;
    if (shift > 15) return 0; 
    return 1<<shift;
}

int main() {
    double frequency;
    uint16_t display;

    stdio_init_all();
    sleep_ms(5000);
    printf("\nfreq_to_display\n");

    init_led();

    init_i2c();

    init_irq();

    // Wait forever
    while (true) {
        start_search_safe();

        toggle_led();

        // wait some time to compute period
        // the wait time must be longer than the time for detecting the lower frequency 
        sleep_ms(SLEEP_TIME_MS);

        if (done) {
            frequency = LEN * 1000000.0 / period;
            if (frequency > MAX_FREQ) {
                printf("Too high   \r");
            }
            else if (frequency < MIN_FREQ) {
                printf("Too low    \r");
            }
            else {
                printf(" %8.2f  \r", frequency);
                display = frequency_to_display(frequency);
                refresh_i2c_display(display);
            }
        }
        else {
            if (first) {
                printf("No signal  \r");
            }
            else {
                printf("Too low    \r");
            }
        }
    }
}