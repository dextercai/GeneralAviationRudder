//
// Created by dextercai on 25-6-28.
//

#include "mcp3208.h"

#include <sys/types.h>

void MCP3208_Init(MCP3208* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin) {
    dev->csPin = csPin;
    dev->hspi = hspi;
    dev->csPort = csPort;
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
}

uint16_t MCP3208_ReadChannel(MCP3208* dev, uint8_t channel) {
    if (channel > 7) return 0;
    uint8_t command1 = 0b00000110 | ((channel & 0x04) >> 2); // Start + Single + D2
    uint8_t command2 = (uint8_t)((channel & 0x03) << 6);     // D1, D0 在高位
    uint8_t tx[3] = {command1, command2, 0x00};
    uint8_t rx[3] = {0};
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_RESET);
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(dev->hspi, (uint8_t*)tx, rx, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(dev->csPort, dev->csPin, GPIO_PIN_SET);
    if (status != HAL_OK) {
        return 0;
    }
    uint16_t value = ((rx[1] & 0x0F) << 8) | rx[2];
    return value;
}