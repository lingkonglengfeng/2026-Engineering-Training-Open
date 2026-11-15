#ifndef W25Q64_H
#define W25Q64_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define W25Q16_CAPACITY_BYTES       (2UL * 1024UL * 1024UL)
#define W25Q64_CAPACITY_BYTES       (8UL * 1024UL * 1024UL)
#define W25Q64_PAGE_SIZE            256UL
#define W25Q64_SECTOR_SIZE          4096UL
#define W25Q64_BLOCK32_SIZE         (32UL * 1024UL)
#define W25Q64_BLOCK64_SIZE         (64UL * 1024UL)
#define W25Q16_EXPECTED_JEDEC_ID    0xEF4015UL
#define W25Q64_EXPECTED_JEDEC_ID    0xEF4017UL

typedef enum
{
    W25Q64_OK = 0,
    W25Q64_ERROR_PARAM,
    W25Q64_ERROR_SPI,
    W25Q64_ERROR_TIMEOUT,
    W25Q64_ERROR_ID,
    W25Q64_ERROR_VERIFY
} w25q64_status_t;

typedef struct
{
    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_id;
} w25q64_id_t;

w25q64_status_t W25Q64_Init(w25q64_id_t *id);
/* Serialize complete storage transactions across RTOS owners. Nonblocking. */
uint8_t W25Q64_StorageTryLock(void);
void W25Q64_StorageUnlock(void);
uint32_t W25Q64_GetCapacity(void);
w25q64_status_t W25Q64_ReadJedecId(w25q64_id_t *id);
w25q64_status_t W25Q64_ReadStatus1(uint8_t *status);
w25q64_status_t W25Q64_WaitReady(uint32_t timeout_ms);
w25q64_status_t W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length);
w25q64_status_t W25Q64_Write(uint32_t address, const uint8_t *data,
                             uint32_t length);
w25q64_status_t W25Q64_WriteVerified(uint32_t address, const uint8_t *data,
                                     uint32_t length, uint8_t *verify_buffer,
                                     uint32_t verify_buffer_size);
w25q64_status_t W25Q64_EraseSector(uint32_t address);
w25q64_status_t W25Q64_EraseBlock32(uint32_t address);
w25q64_status_t W25Q64_EraseBlock64(uint32_t address);
w25q64_status_t W25Q64_EraseChip(void);
w25q64_status_t W25Q64_PowerDown(void);
w25q64_status_t W25Q64_WakeUp(void);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */
