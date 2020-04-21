#include "main.h"
#include "crc.h"

CRC_HandleTypeDef hcrc;

void CRC_Init() {
    __HAL_RCC_CRC_CLK_ENABLE();
    HAL_CRC_Init(&hcrc);
}

void CRC_DeInit() {
    __HAL_RCC_CRC_CLK_DISABLE();
    HAL_CRC_DeInit(&hcrc);
}

uint32_t calc_crc32(uint32_t* address, uint32_t size_in_byte)
{
    return ~(HAL_CRC_Calculate(&hcrc, address, size_in_byte));
}
