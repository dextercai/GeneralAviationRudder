//
// Created by dextercai on 25-6-28.
//

#ifndef MCP3208_H
#define MCP3208_H

#include "stm32f1xx_hal.h"

// #define MCP3208_ERROR_INVALID_CHANNEL   (-1)
// #define MCP3208_ERROR_SPI_HAL_FAIL      (-2)

typedef struct {
    SPI_HandleTypeDef* hspi;
    GPIO_TypeDef* csPort;
    uint16_t csPin;
} MCP3208;

void MCP3208_Init(MCP3208* dev, SPI_HandleTypeDef* hspi, GPIO_TypeDef* csPort, uint16_t csPin);
uint16_t MCP3208_ReadChannel(MCP3208* dev, uint8_t channel);


#endif //MCP3208_H
