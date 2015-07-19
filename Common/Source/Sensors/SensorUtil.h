/* Sensor Utility */
#include <jendefs.h>
#include <string.h>
#include "SMBus.h"

#define ZeroMemory(Destination,Length) memset((Destination),0,(Length))

uint16 CRC16Calc(uint8 *pu8Data, uint8 len);
bool_t WriteI2CSingleByte(uint8 addr, uint8 cmd, uint8 data);
uint8 ReadI2CSingleByte(uint8 addr, uint8 cmd);
bool_t ReadI2CMultiByte(uint8 addr, uint8 cmd, uint8 length, uint8 *output);
