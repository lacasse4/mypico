#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include "fedge.h"

// variables used in falling_edge_handler()
static volatile int irq_state = FEH_IDLE;       // quiet at startup
static volatile uint32_t irq_start;
static volatile uint32_t irq_period;
static volatile int irq_counter;
static uint irq_input_pin;
static int irq_length;

// Interrupt handler on INPUT_PIN. Detects the period between two falling edges.
// To arm the counter, call arm_falling_edge_handler()
// To check the counter status, call is_falling_edge_handler_done()
void falling_edge_handler(uint gpio, uint32_t events) 
{
    // disable interrupts from INPUT_PIN during the execution of this routine
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, false);

    uint32_t time_stamp = time_us_32();

    // exit if in IDLE or DONE state
    if (irq_state == FEH_DONE && irq_state == FEH_IDLE) return;

    if (irq_state == FEH_COUNTING) {
        irq_counter++;
        if (irq_counter == irq_length) {
            irq_period = time_stamp - irq_start;
            irq_state = FEH_DONE;
        }
    }

    if (irq_state == FEH_LAUNCH) {
        irq_start = time_stamp;
        irq_state = FEH_COUNTING;
    }

    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Setup the interrupt handler on PIN_IRQ
void init_fedge_handler(uint input_pin, int length) 
{
    irq_input_pin = input_pin;
    irq_length = length;
    gpio_set_input_enabled (irq_input_pin, true);
    gpio_pull_up (irq_input_pin);
    gpio_set_irq_enabled_with_callback(irq_input_pin, GPIO_IRQ_EDGE_FALL, true, &falling_edge_handler);
}

// Arm the the counter safely
void arm_fedge_handler() 
{
    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, false);
    irq_state = FEH_LAUNCH;
    irq_period = 0;
    irq_counter = 0;
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, true);
}

// Check if we went though LENGTH falling edges
bool is_fedge_handler_done()
{
    int state;

    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, false);
    state = irq_state;
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, true);

    return state == FEH_DONE;
}

// Get the frequency from falling_edge_handler() data
// Note: check that is_falling_edge_handler_done() == FEH_DONE before performing this call.
double get_fedge_frequency() 
{
    int state;
    uint32_t period;

    // disable interrupts from PIN_IRQ while accessing share variables
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, false);
    state = irq_state;
    period = irq_period;
    gpio_set_irq_enabled (irq_input_pin, GPIO_IRQ_EDGE_FALL, true);

    if (state == FEH_DONE) return irq_length * 1000000.0 / period;
    else return 0.0;
}

