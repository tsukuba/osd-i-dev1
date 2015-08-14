/* Sensor Utility */
#ifndef SENSORUTIL_H_
#define SENSORUTIL_H_
#include <jendefs.h>
#include <string.h>
#include "SMBus.h"

#define ZeroMemory(Destination,Length) memset((Destination),0,(Length))

uint16 CRC16Calc(uint8 *pu8Data, uint8 len);
uint16 CRC16_CCITT(uint8 *data, uint8 len);
bool_t WriteI2CSingleByte(uint8 addr, uint8 cmd, uint8 data);
uint8 ReadI2CSingleByte(uint8 addr, uint8 cmd);
bool_t ReadI2CMultiByte(uint8 addr, uint8 cmd, uint8 length, uint8 *output);
#endif /* SENSORUTIL_H_ */
