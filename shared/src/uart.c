#include "uart.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

// #define UART_WRITE_FLAG(value, shift, mask)         
// #define UART_READ_FLAG     

#define BAUD_RATE  115200

void usart2_isr() {

    

}


void uart_setup() {

    //set usart clock
    rcc_periph_clock_enable(RCC_USART2);

    //enable usart 
    USART2_CR1 |= USART_CR1_UE;

    //set data size to 8 bits (1 start bit, 8 data bits, n stop bits)
    //stops bits is configured in next step
    USART2_CR1 &= ~(USART_CR1_M);

    //configure stop bits[12:13] (set to 00 for 1 stop bit)
    USART2_CR2 &= ~(3 << 12);

    //set baud rate for tx and rx
    uint32_t clock = rcc_get_usart_clk_freq(USART2);
    USART2_BRR = (clock + BAUD_RATE / 2) / BAUD_RATE;

    //enable interrupt when RXNE=1 or ORE = 1
    USART2_CR1 |= USART_CR1_RXNEIE;

    //enable receiver
    USART2_CR1 |= USART_CR1_RE;

}

void uart_write(uint8_t* data, uint32_t length) {

    //enable transmission for uart by setting TE bit
    USART2_CR1 |= USART_CR1_TE;

    for(int i = 0; i < length; i++) {
        uart_write_byte(*(data + i));
    }

    //wait for tc bit to set
    while ( ((USART2_CR1 >> 6) & 1) != 1) {

    }

    //disable transmission
    USART2_CR1 &= ~USART_CR1_TE;
}

void uart_write_byte(uint8_t data) {
    USART2_DR = data;
}

void uart_read(uint8_t* data, uint32_t length) {

    for (int i = 0; i < length; i++) {
        *(data + i) = uart_read_byte();
    }

}

uint8_t uart_read_byte() {
    uint8_t byte = USART2_DR;
    return byte;
}