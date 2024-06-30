#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

// Define MCP23018 registers 
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

#define I2C_ADDRESS 0x20

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx
bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int main() {
    int ret;
    uint8_t opcode;
    uint8_t data[2];

    // Enable UART so we can print status output
    stdio_init_all();
    sleep_ms(5000);
    printf("i2c_mcp23018\n");

    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
    i2c_init(i2c_default, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    // bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // set MCP23018 IOCON register
    //   - set sequencial mode (bit 5 to 0)
    //   - set default interrupt pin level to LOW (bit 2 to 1) 
    opcode = IOCON;
    data[0] = 0x02;
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
    printf("write iocon opcode ret= %d\n", ret);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 1, false);
    printf("write iocon data ret= %d\n", ret);

    // set MCP23018 port A and B as outputs 
    opcode = IODIRA;
    data[0] = 0x00;
    data[1] = 0x00;
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
    printf("write iodira opcode ret= %d\n", ret);
    ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 2, false);
    printf("write iodira data ret= %d\n", ret);

    int counter = 0;
    uint16_t shift = 1;
    while(1) {
        uint16_t value = ~shift;
        opcode = GPIOA;
        data[0] = (uint8_t)(value & 0x00FF);
        data[1] = (uint8_t)((value >> 8) & 0x00FF);
        ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, &opcode, 1, true);
        ret = i2c_write_blocking (i2c_default, I2C_ADDRESS, data, 2, false);
        sleep_ms(100);
        shift <<= 1;
        counter++;
        if (counter >= 15) {
            counter = 0;
            shift = 1;
        }
    }
    return 0;
}
