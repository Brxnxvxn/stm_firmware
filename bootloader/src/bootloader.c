#include "common_defines.h"
#include <libopencm3/stm32/memorymap.h>

#define BOOTLOADER_SIZE         (0x8000U)
#define MAIN_APP_START_ADDR     (FLASH_BASE + BOOTLOADER_SIZE)


static void jump_to_main(void) {

    typedef void (*void_fn)(void);

    uint32_t* reset_vector_entry = (uint32_t*)(MAIN_APP_START_ADDR + 4U);
    uint32_t* reset_vector = (uint32_t*)(*reset_vector_entry);

    //convert address of reset vector into a fucntion pointer so that we can call it as a function
    void_fn jump_fn = (void_fn)reset_vector;

    jump_fn();
}

int main(void) {
    jump_to_main();
    //this function will never return
    return 0;

}