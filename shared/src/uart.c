#include "uart.h"
#include "ring_buffer.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>

// #define UART_WRITE_FLAG(value, shift, mask)         
// #define UART_READ_FLAG     

#define BAUD_RATE  115200
#define BUFFER_SIZE   128U


static bool data_available = false;
static uint8_t data_buf[BUFFER_SIZE] = {0u};
static ring_buffer_t rb = {0U};

void usart2_isr(void) {

    data_available = true;

    //check if ORE bit or RNXE bit is set
    const bool byte_recv = usart_get_flag(USART2, USART_FLAG_RXNE) == 1;
    const bool overrun_error = usart_get_flag(USART2, USART_FLAG_ORE) == 1;

    if (byte_recv || overrun_error) {
        ring_buffer_write(&rb, (uint8_t)usart_recv(USART2));
    }

}

void uart_setup() {

    //set usart clock
    rcc_periph_clock_enable(RCC_USART2);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    usart_set_databits(USART2, 8);
    usart_set_baudrate(USART2, BAUD_RATE);
    usart_set_parity(USART2, 0);
    usart_set_stopbits(USART2, 1);
    
    usart_enable_rx_interrupt(USART2);
    nvic_enable_irq(NVIC_USART2_IRQ);
    usart_enable(USART2);

    /* initalize rx ring buffer */
    ring_buffer_init(&rb, data_buf, BUFFER_SIZE);

}

void uart_write(uint8_t* data, uint32_t length) {


    for(uint32_t i = 0; i < length; i++) {
        uart_write_byte(*(data));
    }

}

void uart_write_byte(uint8_t data) {
    usart_send_blocking(USART2, (uint16_t)data);
}

uint32_t uart_read(uint8_t* data, uint32_t length) {

    /* if length is less than or equal to zero, */
    if(length == 0U)
        return 0U;

    for(uint32_t bytes_read = 0U; bytes_read < length; bytes_read++)
    {
        if(!ring_buffer_read(&rb, data + bytes_read))
        {
            return bytes_read; 
        }
    }

    return length;
}

uint8_t uart_read_byte() {
    uint8_t byte = 0;
    uart_read(&byte, 1U);
    return byte;
}

bool uart_data_available() {
    return !ring_buffer_empty(&rb);
}