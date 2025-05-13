/**
 * @name fedge
 * @details
 * Falling edge detection: this module implements functions to detect falling edges on a GPIO pin, 
 * and measures the period elapsed between subsequent falling edges. 
 * The period is converted to a frequency.
 * 
 * Technology:
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 'input_pin' for frequency estimation
 *  - uses an interrupt handler on 'input_pin'
 *  - uses the pico internal timer (1 MHz)
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-02
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "fdetect.h"
#include "fedge.h"

#define MAX_SLOTS (MAX_LENGTH+1)  // we need MAX_LENGTH+1 slots (time stamps) to measure MAX_LENGTH periods

// variables used in fedge_handler()
static volatile int state;       
static volatile uint32_t time_stamp[MAX_SLOTS] = {0};   
static volatile int last;
static volatile int first;
static volatile int counter;
static uint input_pin;
static int length;
static int slots;

// Interrupt handler on input_pin. Detects the period between falling edges.
// - to initialize the module, call fedge_init()
// - to reset and launch falling edge detection, call fedge_launch()
// - to check if valid data is available, call fedge_is_read_ok()
// - to get data, call fedge_get_frequency() after checking fedge_is_read_ok()
void fedge_handler(uint gpio, uint32_t events) 
{
    // disable interrupts from INPUT_PIN during the execution of this routine
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);

    uint32_t now = time_us_32();  // get current time

    if (state == FEH_IDLE) return;

    if (state == FEH_READ_OK) {
        first = (first+1) % slots;
        last  = (last+1)  % slots;
        time_stamp[last] = now;
    }

    if (state == FEH_COUNTING) {
        first = (first+1) % slots;
        last  = (last+1)  % slots;
        time_stamp[last] = now;
        state = FEH_READ_OK;
    }

    if (state == FEH_LOADING) {
        time_stamp[counter++] = now;
        if (counter == slots) {
            state = FEH_READ_OK;
        }
    }

    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Get frequency from falling_edge_handler() data
// Note: fedge_is_read_ok() must be true before performing this call.
double fedge_get_frequency() 
{
    uint32_t period;

    // disable interrupts from input_pin while accessing share variables
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);
    period = time_stamp[last] - time_stamp[first];
    state = FEH_COUNTING;
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);

    // RP2040 system timer runs ar 1 MHz
    return length * 1000000.0 / period;
}

// Init variables and setup interrupt handler on input_pin
void fedge_init(uint _input_pin, int _length) 
{
    input_pin = _input_pin;
    length = _length;
    slots = _length + 1;
    state = FEH_IDLE;           // quiet at the beginning 

    gpio_set_input_enabled (input_pin, true);
    gpio_pull_up (input_pin);
    gpio_set_irq_enabled_with_callback(input_pin, GPIO_IRQ_EDGE_FALL, true, &fedge_handler);
    // gpio_set_input_hysteresis_enabled (input_pin, true);
}

// Arm edge detector 
void fedge_launch() 
{
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);
    last = length;
    first = 0;
    counter = 0;
    state = FEH_LOADING;
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Check if we went though length+1 falling edges
bool fedge_is_read_ok()
{
    return state == FEH_READ_OK;
}

bool fedge_no_falling_edge()
{
    return counter == 0;
    // gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);

    // gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);
}
