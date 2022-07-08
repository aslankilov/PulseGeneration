/* 
binary convertions
alex.sannikov.zero@gmail.com
*/
#ifndef BINARY_SUPPORT
#define BINARY_SUPPORT
#include "stm32f4xx_hal.h"

uint16_t calcCRC(uint8_t *data, uint16_t size);

uint8_t LOW_BYTE(uint16_t trans);
uint8_t HIGH_BYTE(uint16_t trans);
uint16_t uniq(uint8_t highByte, uint8_t lowByte);


float linearDeadZone(float value, float minValue, float maxValue);
float constrain(float value, float low, float high);
#endif
