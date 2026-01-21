#ifndef BL_FLASH_H
#define BL_FLASH_H

#include "common_defines.h"

void flash_erase_main_app(void);
void flash_write(uint32_t address, uint8_t* data, uint32_t size);


#endif /* BL_FLASH_H */