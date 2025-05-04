/**
 * @name fedge
 * @details
 * Falling edge detection: this module implements functions to detect falling edges on a GPIO pin, 
 * and measures the period elapsed between two subsequent falling edges. 
 * The period is converted to a frequency.
 * 
 * Technology:
 *  - runs on a Raspberry pi pico 1 built w/ C/C++ SDK 1.5.1
 *  - detects fallings edges on GPIO 'input_pin' for frequency estimation
 *  - uses an interrupt handler on 'input_pin'
 * 
 * @note  It is assumed that the input signal is analogicaly filtered 
 * with an 8 poles low pass filter that is set with a 1 KHz cut off frequency. 
 *
 * @author Vincent Lacasse
 * @date   2025-05-02
 */

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "fedge.h"

// variables used in falling_edge_handler()
static volatile int state = FEH_IDLE;       // quiet at startup
static volatile uint32_t start;
static volatile uint32_t period;
static volatile int counter;
static uint input_pin;

// Interrupt handler on input_pin. Detects the period between two falling edges.
// To arm the counter, call arm_falling_edge_handler()
// To check the counter status, call is_falling_edge_handler_done()
void falling_edge_handler(uint gpio, uint32_t events) 
{
    // disable interrupts from INPUT_PIN during the execution of this routine
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);

    uint32_t time_stamp = time_us_32();

    // exit if in IDLE or DONE state
    if (state == FEH_DONE && state == FEH_IDLE) return;

    if (state == FEH_COUNTING) {
        counter++;
        if (counter == LENGTH) {
            period = time_stamp - start;
            state = FEH_DONE;
        }
    }

    if (state == FEH_LAUNCH) {
        start = time_stamp;
        state = FEH_COUNTING;
    }

    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Setup the interrupt handler on input_pin
void init_fedge_handler(uint _input_pin) 
{
    input_pin = _input_pin;
    gpio_set_input_enabled (input_pin, true);
    gpio_pull_up (input_pin);
    gpio_set_irq_enabled_with_callback(input_pin, GPIO_IRQ_EDGE_FALL, true, &falling_edge_handler);
}

// Arm the the counter safely
void arm_fedge_handler() 
{
    // disable interrupts from input_pin while accessing share variables
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);
    state = FEH_LAUNCH;
    period = 0;
    counter = 0;
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Check if we went though LENGTH falling edges
bool is_fedge_handler_done()
{
    return state == FEH_DONE;
}

// Get the frequency from falling_edge_handler() data
// Note: check that is_falling_edge_handler_done() == FEH_DONE before performing this call.
double get_fedge_frequency() 
{
    int _state;
    uint32_t _period;

    // disable interrupts from input_pin while accessing share variables
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, false);
    _state = state;
    _period = period;
    gpio_set_irq_enabled (input_pin, GPIO_IRQ_EDGE_FALL, true);

    if (_state == FEH_DONE) return LENGTH * 1000000.0 / _period;
    else return 0.0;
}

