// I2C
#include "SensorUtil.h"

uint16 CRC16Calc(uint8 *pu8Data, uint8 len) {
	uint16 crc = 0xFFFF;
	uint8 i;

	while (len--) {
		crc ^= *pu8Data++;
		for (i = 0; i < 8; i++) {
			if ((crc & 0x01) != 0) {
				crc >>= 1;
				crc ^= 0xA001;
			} else {
				crc >>= 1;
			}
		}
	}

	return crc;
}

// 指定したコマンドを書くだけ
bool_t WriteI2CSingleByte(uint8 addr, uint8 cmd, uint8 data) {
	return  bSMBusWrite(addr, cmd, 1, &data);
}

uint8 ReadI2CSingleByte(uint8 addr, uint8 cmd) {
	uint8 output[8];
	bool_t bOk = TRUE;
	ZeroMemory(output, sizeof(output));

	bOk &= bSMBusWrite(addr, cmd, 0, NULL);
	bOk &= bSMBusSequentialRead(addr, 1, output);

	if (bOk != TRUE) return 0;
	return output[0];
}

// 指定したコマンドを実行してLength分読む
bool_t ReadI2CMultiByte(uint8 addr, uint8 cmd, uint8 length, uint8 *output) {
	bool_t bOk = TRUE;
	bOk &= bSMBusWrite(addr, cmd, 0, NULL);
	bOk &= bSMBusSequentialRead(addr, length, output);
	return bOk;
}
