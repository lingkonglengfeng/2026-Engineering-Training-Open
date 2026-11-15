#include "w25q64.h"

#include <string.h>

#include "main.h"
#include "spi.h"

#define W25Q64_CMD_WRITE_ENABLE       0x06U
#define W25Q64_CMD_READ_STATUS1       0x05U
#define W25Q64_CMD_READ_JEDEC_ID      0x9FU
#define W25Q64_CMD_FAST_READ          0x0BU
#define W25Q64_CMD_PAGE_PROGRAM       0x02U
#define W25Q64_CMD_SECTOR_ERASE       0x20U
#define W25Q64_CMD_BLOCK_ERASE_32K    0x52U
#define W25Q64_CMD_BLOCK_ERASE_64K    0xD8U
#define W25Q64_CMD_CHIP_ERASE         0xC7U
#define W25Q64_CMD_POWER_DOWN         0xB9U
#define W25Q64_CMD_RELEASE_POWER_DOWN 0xABU

#define W25Q64_STATUS_BUSY            0x01U
#define W25Q64_STATUS_WEL             0x02U

#define W25Q64_SPI_TIMEOUT_MS         100U
#define W25Q64_PROGRAM_TIMEOUT_MS     10U
#define W25Q64_SECTOR_TIMEOUT_MS      500U
#define W25Q64_BLOCK_TIMEOUT_MS       2000U
#define W25Q64_CHIP_TIMEOUT_MS        100000U

#define W25Q64_CS_LOW()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define W25Q64_CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

static uint32_t s_capacity_bytes;
static volatile uint8_t s_storage_busy;
uint8_t W25Q64_StorageTryLock(void)
{
    uint32_t mask=__get_PRIMASK();uint8_t acquired;
    __disable_irq();acquired=(uint8_t)(!s_storage_busy);if(acquired)s_storage_busy=1U;__set_PRIMASK(mask);return acquired;
}
void W25Q64_StorageUnlock(void)
{uint32_t mask=__get_PRIMASK();__disable_irq();s_storage_busy=0U;__set_PRIMASK(mask);}

static w25q64_status_t W25Q64_Transmit(const uint8_t *data, uint16_t length)
{
    return (HAL_SPI_Transmit(&hspi1, (uint8_t *)data, length,
                             W25Q64_SPI_TIMEOUT_MS) == HAL_OK) ?
           W25Q64_OK : W25Q64_ERROR_SPI;
}

static w25q64_status_t W25Q64_Receive(uint8_t *data, uint16_t length)
{
    return (HAL_SPI_Receive(&hspi1, data, length,
                            W25Q64_SPI_TIMEOUT_MS) == HAL_OK) ?
           W25Q64_OK : W25Q64_ERROR_SPI;
}

static uint8_t W25Q64_RangeValid(uint32_t address, uint32_t length)
{
    return (s_capacity_bytes != 0U) && (length <= s_capacity_bytes) &&
           (address <= (s_capacity_bytes - length));
}

static void W25Q64_BuildAddressCommand(uint8_t *command, uint8_t opcode,
                                       uint32_t address)
{
    command[0] = opcode;
    command[1] = (uint8_t)(address >> 16U);
    command[2] = (uint8_t)(address >> 8U);
    command[3] = (uint8_t)address;
}

static w25q64_status_t W25Q64_WriteEnable(void)
{
    uint8_t command = W25Q64_CMD_WRITE_ENABLE;
    uint8_t status;
    w25q64_status_t result;

    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    W25Q64_CS_HIGH();
    if (result != W25Q64_OK)
    {
        return result;
    }

    result = W25Q64_ReadStatus1(&status);
    if (result != W25Q64_OK)
    {
        return result;
    }
    return ((status & W25Q64_STATUS_WEL) != 0U) ? W25Q64_OK : W25Q64_ERROR_SPI;
}

static w25q64_status_t W25Q64_Erase(uint8_t command_code, uint32_t address,
                                    uint32_t alignment, uint32_t timeout_ms)
{
    uint8_t command[4];
    w25q64_status_t result;

    if ((s_capacity_bytes == 0U) || (address >= s_capacity_bytes) ||
        ((address % alignment) != 0U))
    {
        return W25Q64_ERROR_PARAM;
    }
    result = W25Q64_WaitReady(W25Q64_SPI_TIMEOUT_MS);
    if (result != W25Q64_OK) { return result; }
    result = W25Q64_WriteEnable();
    if (result != W25Q64_OK) { return result; }

    W25Q64_BuildAddressCommand(command, command_code, address);
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(command, sizeof(command));
    W25Q64_CS_HIGH();
    if (result != W25Q64_OK) { return result; }
    return W25Q64_WaitReady(timeout_ms);
}

w25q64_status_t W25Q64_ReadStatus1(uint8_t *status)
{
    uint8_t command = W25Q64_CMD_READ_STATUS1;
    w25q64_status_t result;

    if (status == NULL) { return W25Q64_ERROR_PARAM; }
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    if (result == W25Q64_OK) { result = W25Q64_Receive(status, 1U); }
    W25Q64_CS_HIGH();
    return result;
}

w25q64_status_t W25Q64_WaitReady(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t status;
    w25q64_status_t result;

    do
    {
        result = W25Q64_ReadStatus1(&status);
        if (result != W25Q64_OK) { return result; }
        if ((status & W25Q64_STATUS_BUSY) == 0U) { return W25Q64_OK; }
    } while ((HAL_GetTick() - start) < timeout_ms);
    return W25Q64_ERROR_TIMEOUT;
}

w25q64_status_t W25Q64_ReadJedecId(w25q64_id_t *id)
{
    uint8_t command = W25Q64_CMD_READ_JEDEC_ID;
    uint8_t raw_id[3];
    w25q64_status_t result;

    if (id == NULL) { return W25Q64_ERROR_PARAM; }
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    if (result == W25Q64_OK) { result = W25Q64_Receive(raw_id, sizeof(raw_id)); }
    W25Q64_CS_HIGH();
    if (result != W25Q64_OK) { return result; }
    id->manufacturer_id = raw_id[0];
    id->memory_type = raw_id[1];
    id->capacity_id = raw_id[2];
    return W25Q64_OK;
}

w25q64_status_t W25Q64_Init(w25q64_id_t *id)
{
    w25q64_id_t local_id;
    uint32_t jedec_id;
    w25q64_status_t result;

    W25Q64_CS_HIGH();
    HAL_Delay(1U);
    result = W25Q64_WakeUp();
    if (result != W25Q64_OK) { return result; }
    result = W25Q64_WaitReady(W25Q64_SPI_TIMEOUT_MS);
    if (result != W25Q64_OK) { return result; }
    result = W25Q64_ReadJedecId(&local_id);
    if (result != W25Q64_OK) { return result; }
    jedec_id = ((uint32_t)local_id.manufacturer_id << 16U) |
               ((uint32_t)local_id.memory_type << 8U) |
               local_id.capacity_id;
    if (id != NULL) { *id = local_id; }
    if (jedec_id == W25Q16_EXPECTED_JEDEC_ID)
    {
        s_capacity_bytes = W25Q16_CAPACITY_BYTES;
        return W25Q64_OK;
    }
    if (jedec_id == W25Q64_EXPECTED_JEDEC_ID)
    {
        s_capacity_bytes = W25Q64_CAPACITY_BYTES;
        return W25Q64_OK;
    }
    s_capacity_bytes = 0U;
    return W25Q64_ERROR_ID;
}

uint32_t W25Q64_GetCapacity(void)
{
    return s_capacity_bytes;
}

w25q64_status_t W25Q64_Read(uint32_t address, uint8_t *data, uint32_t length)
{
    uint8_t command[5];
    uint32_t chunk;
    w25q64_status_t result;

    if ((length == 0U) || (data == NULL) || !W25Q64_RangeValid(address, length))
    {
        return W25Q64_ERROR_PARAM;
    }
    result = W25Q64_WaitReady(W25Q64_SPI_TIMEOUT_MS);
    if (result != W25Q64_OK) { return result; }
    W25Q64_BuildAddressCommand(command, W25Q64_CMD_FAST_READ, address);
    command[4] = 0U;
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(command, sizeof(command));
    while ((result == W25Q64_OK) && (length > 0U))
    {
        chunk = (length > 0xFFFFU) ? 0xFFFFU : length;
        result = W25Q64_Receive(data, (uint16_t)chunk);
        data += chunk;
        length -= chunk;
    }
    W25Q64_CS_HIGH();
    return result;
}

w25q64_status_t W25Q64_Write(uint32_t address, const uint8_t *data,
                             uint32_t length)
{
    uint8_t command[4];
    uint32_t page_space;
    uint16_t chunk;
    w25q64_status_t result;

    if ((length == 0U) || (data == NULL) || !W25Q64_RangeValid(address, length))
    {
        return W25Q64_ERROR_PARAM;
    }
    while (length > 0U)
    {
        page_space = W25Q64_PAGE_SIZE - (address % W25Q64_PAGE_SIZE);
        chunk = (uint16_t)((length < page_space) ? length : page_space);
        result = W25Q64_WaitReady(W25Q64_PROGRAM_TIMEOUT_MS);
        if (result != W25Q64_OK) { return result; }
        result = W25Q64_WriteEnable();
        if (result != W25Q64_OK) { return result; }
        W25Q64_BuildAddressCommand(command, W25Q64_CMD_PAGE_PROGRAM, address);
        W25Q64_CS_LOW();
        result = W25Q64_Transmit(command, sizeof(command));
        if (result == W25Q64_OK) { result = W25Q64_Transmit(data, chunk); }
        W25Q64_CS_HIGH();
        if (result != W25Q64_OK) { return result; }
        result = W25Q64_WaitReady(W25Q64_PROGRAM_TIMEOUT_MS);
        if (result != W25Q64_OK) { return result; }
        address += chunk;
        data += chunk;
        length -= chunk;
    }
    return W25Q64_OK;
}

w25q64_status_t W25Q64_WriteVerified(uint32_t address, const uint8_t *data,
                                     uint32_t length, uint8_t *verify_buffer,
                                     uint32_t verify_buffer_size)
{
    uint32_t offset = 0U;
    uint32_t chunk;
    w25q64_status_t result;

    if ((verify_buffer == NULL) || (verify_buffer_size == 0U))
    {
        return W25Q64_ERROR_PARAM;
    }
    result = W25Q64_Write(address, data, length);
    if (result != W25Q64_OK) { return result; }
    while (offset < length)
    {
        chunk = length - offset;
        if (chunk > verify_buffer_size) { chunk = verify_buffer_size; }
        result = W25Q64_Read(address + offset, verify_buffer, chunk);
        if (result != W25Q64_OK) { return result; }
        if (memcmp(&data[offset], verify_buffer, chunk) != 0)
        {
            return W25Q64_ERROR_VERIFY;
        }
        offset += chunk;
    }
    return W25Q64_OK;
}

w25q64_status_t W25Q64_EraseSector(uint32_t address)
{
    return W25Q64_Erase(W25Q64_CMD_SECTOR_ERASE, address,
                        W25Q64_SECTOR_SIZE, W25Q64_SECTOR_TIMEOUT_MS);
}

w25q64_status_t W25Q64_EraseBlock32(uint32_t address)
{
    return W25Q64_Erase(W25Q64_CMD_BLOCK_ERASE_32K, address,
                        W25Q64_BLOCK32_SIZE, W25Q64_BLOCK_TIMEOUT_MS);
}

w25q64_status_t W25Q64_EraseBlock64(uint32_t address)
{
    return W25Q64_Erase(W25Q64_CMD_BLOCK_ERASE_64K, address,
                        W25Q64_BLOCK64_SIZE, W25Q64_BLOCK_TIMEOUT_MS);
}

w25q64_status_t W25Q64_EraseChip(void)
{
    uint8_t command = W25Q64_CMD_CHIP_ERASE;
    w25q64_status_t result;

    result = W25Q64_WaitReady(W25Q64_SPI_TIMEOUT_MS);
    if (result != W25Q64_OK) { return result; }
    result = W25Q64_WriteEnable();
    if (result != W25Q64_OK) { return result; }
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    W25Q64_CS_HIGH();
    if (result != W25Q64_OK) { return result; }
    return W25Q64_WaitReady(W25Q64_CHIP_TIMEOUT_MS);
}

w25q64_status_t W25Q64_PowerDown(void)
{
    uint8_t command = W25Q64_CMD_POWER_DOWN;
    w25q64_status_t result;

    result = W25Q64_WaitReady(W25Q64_SPI_TIMEOUT_MS);
    if (result != W25Q64_OK) { return result; }
    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    W25Q64_CS_HIGH();
    return result;
}

w25q64_status_t W25Q64_WakeUp(void)
{
    uint8_t command = W25Q64_CMD_RELEASE_POWER_DOWN;
    w25q64_status_t result;

    W25Q64_CS_LOW();
    result = W25Q64_Transmit(&command, 1U);
    W25Q64_CS_HIGH();
    if (result == W25Q64_OK) { HAL_Delay(1U); }
    return result;
}
