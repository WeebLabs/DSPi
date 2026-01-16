#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

#include <stdint.h>

// Result codes
#define FLASH_OK            0
#define FLASH_ERR_WRITE     1
#define FLASH_ERR_NO_DATA   2
#define FLASH_ERR_CRC       3

// Flash storage API
int flash_save_params(void);
int flash_load_params(void);
void flash_factory_reset(void);

#endif // FLASH_STORAGE_H
