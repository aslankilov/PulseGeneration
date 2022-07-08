#include "binary_support.h"

uint16_t calcCRC(uint8_t *data, uint16_t size)
{
    uint16_t crc = 0xFFFF;
		uint8_t i;
    while (size--){
        crc ^= *data++ << 8;
        for (i = 0; i < 8; i++)
            crc = crc & 0x8000 ? (crc << 1) ^ 0x1021 : crc << 1;
        }
   return crc;
}


//16-bit convertions
uint8_t LOW_BYTE(uint16_t trans)
	{return (trans & 0xff);}
uint8_t HIGH_BYTE(uint16_t trans)
	{return ((trans >> 8) & 0xff);}
uint16_t uniq(uint8_t highByte, uint8_t lowByte)
	{return ((uint16_t)highByte << 8) | lowByte;}

float linearDeadZone(float value, float minValue, float maxValue)
{

	if ((minValue<=value)&&(value<=maxValue)) 
	{
		value=0;
	}
	else
	{
		if (value>maxValue)
		{
			value -= maxValue;
		}
		else 
		{
			if (value<minValue)
			{
				value -= minValue;
			}
		}
	}
	return value; 
}

float constrain(float value, float low, float high)
{
	return ((value)<=(low)?(low):((value)>=(high)?(high):(value)));
}
