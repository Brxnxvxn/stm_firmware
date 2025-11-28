#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "core/system.h"
#include "uart.h"

#define LED_PORT        (GPIOA)
#define LED_PIN         (GPIO5)

#define UART_PORT       (GPIOA)
#define RX_PIN          (GPIO3)
#define TX_PIN          (GPIO2)

#define BOOTLOADER_SIZE  (0x8000U)

static void vector_setup(void) {
    SCB_VTOR = BOOTLOADER_SIZE;
}


static void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
    gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);

    //set up gpio for uart
    gpio_mode_setup(UART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, RX_PIN | TX_PIN);
    gpio_set_af(UART_PORT, GPIO_AF7, RX_PIN | TX_PIN);

}

int main(void) {
    vector_setup();
    system_setup();
    gpio_setup();
    uart_setup();

    uint64_t start_time = system_get_ticks();

    while(1) {
        if (system_get_ticks() - start_time >= 1000){
            gpio_toggle(LED_PORT, LED_PIN);
            start_time = system_get_ticks();
        }

        if(uart_data_available()) {
            uint8_t byte = uart_read_byte();
            uint8_t send_data = byte + 1;
            uart_write(&send_data, 1);
        }
    }



    return 0;

}