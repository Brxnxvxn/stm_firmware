#include "common_defines.h"
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "system.h"
#include "uart.h"
#include "comms.h"
#include "crc.h"

#define BOOTLOADER_SIZE         (0x8000U)
#define MAIN_APP_START_ADDR     (FLASH_BASE + BOOTLOADER_SIZE)

#define UART_PORT       (GPIOA)
#define RX_PIN          (GPIO3)
#define TX_PIN          (GPIO2)


static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);

    //set up gpio for uart
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN | TX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, RX_PIN | TX_PIN);

}


static void jump_to_main(void) {

    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDR + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);

    //convert address of reset vector into a fucntion pointer so that we can call it as a function
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

int main(void) {
    system_setup();
    gpio_setup();
    uart_setup();
    comms_setup();

    comms_packet_t packet = {
        .length = 9U,
        .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .crc = 0
    };

    packet.crc = crc8((uint8_t*)&packet, 17);

    while (true)
    {
        uint8_t data = 0x12;
        uart_write(&data, 1);

        if(uart_data_available())
        {
            uint8_t byte = uart_read_byte();
            uint8_t send_data = byte + 1;
            uart_write(&send_data, 1);
        }
        
        system_delay(500);
    }

    jump_to_main();
    //this function will never return
    return 0;

}