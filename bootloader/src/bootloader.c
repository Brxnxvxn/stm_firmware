#include "common_defines.h"
#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "system.h"
#include "uart.h"
#include "comms.h"
#include "crc.h"
#include "bl_flash.h"
#include "timer.h"

#define BOOTLOADER_SIZE         (0x8000U)
#define MAIN_APP_START_ADDR     (FLASH_BASE + BOOTLOADER_SIZE)
#define FW_LENGTH               (1024 * 512) - BOOTLOADER_SIZE

#define UART_PORT       (GPIOA)
#define RX_PIN          (GPIO3)
#define TX_PIN          (GPIO2)


#define SYNC_SEQ0   0xAAU
#define SYNC_SEQ1   0xBBU
#define SYNC_SEQ2   0xCCU
#define SYNC_SEQ3   0xDDU
#define DEVICE_ID   0x10U

/* BL SINGLE-BYTE PACKETS */
#define BL_SYNC_COMPLETE            0x20U
#define BL_UPDATE_REQUEST_REQ       0x64U
#define BL_UPDATE_REQUEST_RESP      0x65U
#define BL_DEVID_REQ                0x34U
#define BL_DEVID_RESP               0x35U
#define BL_FWLEN_REQ                0x44U
#define BL_FWLEN_RESP               0x45U

typedef enum 
{
    BL_DONE,
    BL_SYNC,
    BL_UPDATE_REQ,
    BL_DEVICE_ID_REQ,
    BL_DEVICE_ID_RESP,
    BL_FW_LENGTH_REQ,
    BL_FW_LENGH_RESP,
    BL_ERASE_APP,
    BL_FW_UPDATE
}BL_STATE;


static BL_STATE state = BL_SYNC;

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

static void check_timeout(timer_t* timer)
{
    if (timer_timeout(timer))
    {
        /* call bootloading_fail function*/
    }

}

static bool is_fw_length_packet(comms_packet_t* packet)
{
    if(packet->length != 5)
        return false;
    
    if(packet->data[0] != BL_FWLEN_RESP)
        return false;
    
    for(uint32_t i = 5; i < DATA_LENGTH_MAX; i++)
    {
        if(packet->data[i] != 0xFF)
            return false;
    }

    return true;

}


int main(void) {
    system_setup();
    gpio_setup();
    uart_setup();
    comms_setup();
    
    /* initialize timer */
    timer_t bl_timer = {0};
    timer_setup(&bl_timer, 1000U, 0U);

    uint8_t sync_buf[4] = {0U};
    uint8_t sync_count = 0U;
    comms_packet_t temp_packet;
    comms_packet_t read_packet;

    while (state != BL_DONE)
    {
        /* bootloader state machine */

        switch (state) {
            case BL_SYNC:
                if (uart_data_available())
                {
                    sync_buf[sync_count] = uart_read_byte();

                    /* sync count wrap around after all bytes are read */
                    sync_count = (sync_count + 1) % 4;
                }

                /* check if incoming bytes match sync sequence */
                if(sync_buf[0] == SYNC_SEQ0 && sync_buf[1] == SYNC_SEQ1
                    && sync_buf[2] == SYNC_SEQ2 && sync_buf[3] == SYNC_SEQ3)
                {

                    /* Send ack packet back to host after sync */
                    comms_create_single_byte_packet(&temp_packet, BL_SYNC_COMPLETE);
                    comms_write(&temp_packet);

                    state = BL_UPDATE_REQ;
                    timer_reset(&bl_timer);
                    break;
                }

                check_timeout(&bl_timer);
                break;
            
            case BL_UPDATE_REQ:
                
                /* check for incoming packets */
                comms_update();

                if (comms_packet_available())
                {   
                    /* read packet */
                    comms_read(&read_packet);

                    /* check if host is requesting a firmware update */
                    if (comms_is_single_byte_packet(&read_packet, BL_UPDATE_REQUEST_REQ))
                    {
                        /* send ack for device request */
                        comms_create_single_byte_packet(&temp_packet, BL_UPDATE_REQUEST_RESP);
                        comms_write(&temp_packet);

                        state = BL_DEVICE_ID_REQ;
                        timer_reset(&bl_timer);
                        break;
                    }
                    else 
                    {
                        bootloader_fail();
                    }
                }
                
                check_timeout(&bl_timer);
                break;
            
            case BL_DEVICE_ID_REQ:
                /* send device id request */
                comms_create_single_byte_packet(&temp_packet, BL_DEVID_REQ);
                comms_write(&temp_packet);
                state = BL_DEVICE_ID_RESP;
                break;
            
            case BL_DEVICE_ID_RESP:
                /* wait for device id packet */
                comms_update();
                
                if (comms_packet_available())
                {
                    /* read packet */
                    comms_read(&read_packet);

                    /* check if packet data matches with device id */
                    if (comms_is_single_byte_packet(&read_packet, DEVICE_ID))
                    {
                        
                        comms_create_single_byte_packet(&temp_packet, BL_DEVID_RESP);
                        comms_write(&temp_packet);

                        state = BL_FW_LENGTH_REQ;
                        timer_reset(&bl_timer);
                        break;
                    }
                    else 
                    {
                        bootloader_fail();
                    }

                }

                check_timeout(&bl_timer);
                break;
            
            case BL_FW_LENGTH_REQ:
                /* send firmware length request */
                comms_create_single_byte_packet(&temp_packet, BL_FWLEN_REQ);
                comms_write(&temp_packet);
                state = BL_FW_LENGH_RESP;
                break;

            case BL_FW_LENGH_RESP:

                /* wait for firwmare length packet */
                comms_update();
                
                if (comms_packet_available())
                {

                    /* read packet */
                    comms_read(&read_packet);

                    uint32_t fw_length = read_packet.data[1] 
                                        | (read_packet.data[2] << 8)
                                        | (read_packet.data[2] << 16)
                                        | (read_packet.data[2] << 24);

                    /* verify fw length is less than max size */
                    if(is_fw_length_packet(&read_packet) && fw_length < FW_LENGTH)
                    {

                        state = BL_ERASE_APP;
                        timer_reset(&bl_timer);
                        break;
                    }

                    bootloader_fail();

                }

                check_timeout(&bl_timer);
                break;
            
            case BL_ERASE_APP:
                
                /* erase main app */
                flash_erase_main_app();
                state = BL_FW_UPDATE;
                break;

            case BL_FW_UPDATE:

                /* send fw update request packet */



                
        }


    }


    comms_packet_t packet = {
        .length = 9U,
        .data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF},
        .crc = 0
    };

    packet.crc = crc8((uint8_t*)&packet, 17);

    while (true)
    {
        comms_update();

        if(comms_packet_available())
            comms_read(&packet);
        
        // comms_write(&packet);

        // while(!uart_data_available());
        
        // uint8_t byte = uart_read_byte();
        // uint8_t send_data = byte + 1;
        // uart_write(&send_data, 1);
        
        system_delay(500);
    }

    jump_to_main();
    //this function will never return
    return 0;

}