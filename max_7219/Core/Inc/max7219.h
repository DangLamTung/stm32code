#include "stm32f1xx_hal.h"
//#include <array>

#define CS_PORT GPIOA
#define CS_PIN GPIO_PIN_6

#define CS_H HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);
#define CS_L HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);

#define CLK_PORT GPIOA
#define CLK_PIN GPIO_PIN_5

#define CLK_H HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
#define CLK_L HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);

#define DIN_PORT GPIOA
#define DIN_PIN GPIO_PIN_7

#define DIN_H HAL_GPIO_WritePin(DIN_PORT, DIN_PIN, GPIO_PIN_SET);
#define DIN_L HAL_GPIO_WritePin(DIN_PORT, DIN_PIN, GPIO_PIN_RESET);

void write_data(uint16_t data, uint8_t mode);
