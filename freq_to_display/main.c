/**
 * freq_to_display.c
 *
 * Guitar tuner
 * 
 * Measure frequency on GPIO PIN_IRQ.
 * Display frequency on two 15 LEDs bargraph, and string on a third 6 LEDs bargraph
 * 
 * Some adjustments to PRECISION constants still to be made
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
#include <float.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"

// MCP23018 addresses 
#define I2C_ADDRESS_0 0x20  // pin 15 to GND
#define I2C_ADDRESS_1 0x21  // pin 15 set with voltage divider R1=20K, R2=4.7K, VDD=3.3V (see specs) 
#define I2C_ADDRESS_2 0x27  // pin 15 set to VDD
#define STOP() while(1)

// MCP23018 registers 
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

// Error codes
#define NO_ERROR 0
#define ERROR_WRITE_TO_I2C -1

// number of falling edges counted to evaluate PIN_IRQ frequency
#define LEN 10  

// frequency detection parameters
#define MAX_FREQ    400         // in Hz
#define MIN_FREQ    75          // in Hz
#define TIMER_FREQ  1000000     // in Hz
#define MAX_PERIOD_US  (LEN*TIMER_FREQ/MIN_FREQ)
#define MIN_PERIOD_US  (LEN*TIMER_FREQ/MAX_FREQ)
#define SLEEP_TIME_MS 200  // must be physically greater than MAX_PERIOD_US
#if SLEEP_TIME_MS*1000 <= MAX_PERIOD_US
    #warning SLEEP_TIME_MS must be physically greater than MAX_PERIOD_US
#endif

// frequency detection codes
#define FREQ_VALID       0
#define FREQ_TO_HIGH    -1
#define FREQ_TO_LOW     -2
#define FREQ_NO_SIGNAL  -3

// tuner constants
#define NUM_STRING      6           // number of strings on musical instrument
#define BASE_FREQUENCY  220.0       // base frequency used to calculate cents
#define FREQ_BAR_GRAPH_NUM_LED 15   // number of leds on frequency bargraph (should be a odd number)
#define FREQ_BAR_GRAPH_COARSE_PREC (1.0/30.0)  // frequency bar graph coarse precision
#define FREQ_BAR_GRAPH_FINE_PREC   1.0         // frequency bar graph fine precision

double string_cents[NUM_STRING];   // strings cents from BASE_FREQUENCY

// variables used to switch the Pico on board LED (alive signal)
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
static bool led_on = true;

// variables used in the interrupt handler
const uint PIN_IRQ = 14;
static volatile bool first = false;     // disable irq_handler at startup
static volatile bool done  = true;      // disable irq_handler at startup
static volatile uint32_t start;
static volatile uint32_t period;
static volatile int counter;

// Interrupt handler on PIN_IRQ
// When armed (i.e. done = false, first = true), this routine evaluates 
// the time taken for PIN_IRQ to go through LEN falling egdes. 
// Once the evaluation of 'period' is complete, done is set to true
// and the handler stops counting until it is re-armed by arm_handler_safe()
void irq_handler(uint gpio, uint32_t events) 
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

// Setup the interrupt handler on PIN_IRQ
void init_irq() 
{
    gpio_set_input_enabled (PIN_IRQ, true);
    gpio_pull_up (PIN_IRQ);
    gpio_set_irq_enabled_with_callback(PIN_IRQ, GPIO_IRQ_EDGE_FALL, true, &irq_handler);
}

// Arm the interrupt handler to start counting and perform a single evaluation of 'period'
void arm_handler_safe() 
{
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, false);
    done  = false;
    first = true;
    period = 0;
    counter = 0;
    gpio_set_irq_enabled (PIN_IRQ, GPIO_IRQ_EDGE_FALL, true);
}

// Inititialize Pico on board LED
void init_led() 
{
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);
}

// Toggle Pico on board LED
void toggle_led() 
{
    if (led_on) gpio_put(LED_PIN, 0);
    else        gpio_put(LED_PIN, 1);
    led_on = !led_on;
}

// Initialize i2c communication
void init_i2c() 
{
    // use I2C0 on the default SDA and SCL pins (respectively GP4 and GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);  // I2C speed 100 KHz
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

// Output bit_pattern to the bargraph
// LSB corresponds to the first LED at the left.
int set_bargraph(uint8_t address, uint16_t bit_pattern) 
{
    int ret;
    uint8_t opcode;
    uint8_t data[2];

    bit_pattern = ~bit_pattern;  // a bit value of 0 turn on the led.

    opcode = GPIOA;
    data[0] = (uint8_t)(bit_pattern & 0x00FF);
    data[1] = (uint8_t)((bit_pattern >> 8) & 0x00FF);

    ret = i2c_write_blocking (i2c_default, address, &opcode, 1, true);
    if (ret < 0) return ERROR_WRITE_TO_I2C;
    ret = i2c_write_blocking (i2c_default, address, data, 2, false);
    if (ret < 0) return ERROR_WRITE_TO_I2C;

    return NO_ERROR;
}

// Initialize a MCP23018 at the specified I2C address
// Configure all pins as outputs
// Set all outputs to high so the LEDS are off on power up.
int init_MCP28018(uint8_t address) 
{
    int ret;
    uint8_t opcode;
    uint8_t data[2];

    // set MCP23018 IOCON register
    //   - set sequencial mode (bit 5 to 0)
    //   - set default interrupt pin level to LOW (bit 2 to 1) 
    opcode = IOCON;
    data[0] = 0x02;
    ret = i2c_write_blocking (i2c_default, address, &opcode, 1, true);
    if (ret < 0) return ERROR_WRITE_TO_I2C;
    ret = i2c_write_blocking (i2c_default, address, data, 1, false);
    if (ret < 0) return ERROR_WRITE_TO_I2C;

    // set MCP23018 port A and B as outputs 
    opcode = IODIRA;
    data[0] = 0x00;
    data[1] = 0x00;
    ret = i2c_write_blocking (i2c_default, address, &opcode, 1, true);
    if (ret < 0) return ERROR_WRITE_TO_I2C;
    ret = i2c_write_blocking (i2c_default, address, data, 2, false);
    if (ret < 0) return ERROR_WRITE_TO_I2C;

    // init outputs to high state.
    ret = set_bargraph(address, 0);
    if (ret < 0) return ERROR_WRITE_TO_I2C;
    return NO_ERROR;
}

// Show cents value on a LED bargraph
//   address   - I2C address of the MCP23018 driving the bar graph
//   cents     - relative cent value displayed on the bar graph
//   precison  - precision at which the cent value is displayed on the bar graph
//               i.e. if precision is 1.0, the span is 1 cent per LED
//                    if procision is 0.1, the span is 10 cent per LED
// if the cents*precision exceeds the bargraph, nothing is displayed
int show_cents(uint8_t address, double cents, double precision)
{
    int ret;
    uint16_t bit_pattern;
    int shift;

    shift = (int)round(cents * precision) + 7;
    if (shift >= 0 && shift <= FREQ_BAR_GRAPH_NUM_LED)
        bit_pattern = 1<<shift;
    else 
        bit_pattern = 0;

    ret = set_bargraph(address, bit_pattern);
    return ret;
}

// Compute cents value of a frequency from BASE_FREQUENCY
double get_cents(double frequency)
{
    return 1200.0 * logf(frequency/BASE_FREQUENCY) / M_LN2;
}

// Initialize string_cents[] array with standard guitar tuning in cents 
// from A3 (220 Hz) defined as BASE_FREQUENCY
void init_cents() 
{
    string_cents[0] = -1700.0;      // E2
    string_cents[1] = -1200.0;      // A2
    string_cents[2] =  -700.0;      // D3
    string_cents[3] =  -200.0;      // G3
    string_cents[4] =  +200.0;      // B3
    string_cents[5] =  +700.0;      // E4
}

// Return the string that is the nearest the specified cents value
// String 0 is the low E, string 1 is A, on so on to string 5 (high E)
int find_nearest_frequency(double cents)
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


int main() {
    int string, status, freq_code;
    double frequency, cents, cents_delta;

    stdio_init_all();
    printf("\nfreq_to_display\n");

    // Inititialize on board LED (live signal)
    init_led();     

    // Initialize i2c communication
    init_i2c();     

    // Initialize MCP23018 chips
    status = init_MCP28018(I2C_ADDRESS_0);
    if (status == ERROR_WRITE_TO_I2C) {
        printf("Error writinf to I2C address %x2\n", I2C_ADDRESS_0);
        STOP();
    }
    status = init_MCP28018(I2C_ADDRESS_1);
    if (status == ERROR_WRITE_TO_I2C) {
        printf("Error writinf to I2C address %x2\n", I2C_ADDRESS_1);
        STOP();
    }
    status = init_MCP28018(I2C_ADDRESS_2);
    if (status == ERROR_WRITE_TO_I2C) {
        printf("Error writinf to I2C address %x2\n", I2C_ADDRESS_1);
        STOP();
    }

    // Initialize cents array 
    init_cents();

    // Setup interrupt routine on PIN_IRQ
    init_irq();     

    while (true) {
        // Start a single evaluation of 'period'
        arm_handler_safe();

        // Toggle Pico on board LED
        toggle_led();           

        // Wait some time to compute input waveform period.
        // SLEEP_TIME must be greater than the time required to detect the lowest frequency (MAX_PERIOD)
        sleep_ms(SLEEP_TIME_MS);

        // Update displays if the the evaluation of 'period' is done
        if (done) {
            frequency = LEN * 1000000.0 / period;
            if (frequency > MAX_FREQ) {
                freq_code = FREQ_TO_HIGH;
            }
            else if (frequency < MIN_FREQ) {
                freq_code = FREQ_TO_LOW;
            }
            else {
                freq_code = FREQ_VALID;
                cents = get_cents(frequency);
                string = find_nearest_frequency(cents);
                cents_delta = cents - string_cents[string];
                show_cents(I2C_ADDRESS_0, cents_delta, FREQ_BAR_GRAPH_COARSE_PREC);
                show_cents(I2C_ADDRESS_1, cents_delta, FREQ_BAR_GRAPH_FINE_PREC);
                set_bargraph(I2C_ADDRESS_2, 1<<string);
            } 
        }
        else {
            if (first) {
                freq_code = FREQ_NO_SIGNAL;
            }
            else {
                freq_code = FREQ_TO_LOW;
            }
        }

        // printed message to UART
        switch (freq_code) {
            case FREQ_VALID:
                printf(" %8.2f  %d  %8.2f  %8.2f     \r", frequency, string, cents, cents_delta);
                break;

            case FREQ_NO_SIGNAL:
                printf("No signal  \r");
                break;

            case FREQ_TO_LOW:
                printf("Too low    \r");
                break;

            case FREQ_TO_HIGH:
                printf("Too high   \r");
                break;
        }
    }
}