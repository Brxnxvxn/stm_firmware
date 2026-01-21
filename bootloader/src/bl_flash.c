#include "bl_flash.h"

#include <libopencm3/stm32/flash.h>

/* Symbolic constants */
#define MAIN_APP_SECTOR_START   2U
#define MAIN_APP_SECTOR_END     7U

void flash_erase_main_app(void)
{
    /* unlock flash */
    flash_unlock();

    /* erase main app sectors (2 to 7) in flash */
    for (uint8_t sector = MAIN_APP_SECTOR_START; sector <= MAIN_APP_SECTOR_END; sector++)
    {
        flash_erase_sector(sector , FLASH_CR_PROGRAM_X32);
    }

    /* lock flash after erasing sector */
    flash_lock();

}

void flash_write(uint32_t address, uint8_t* data, uint32_t size)
{
    /* unlock flash */
    flash_unlock();

    /* write to flash */
    flash_program(address, data, size);

    /* lock flash after erasing sector */
    flash_lock();

}
