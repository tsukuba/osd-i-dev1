/* AM2320 Source */
#include "AM2320.h"

bool_t GetData_AM2320(AM2320Data *data) {
	uint8 Cmd[2];
	uint8 Read[8];
	bool_t bOk = TRUE;

	ZeroMemory(Cmd, sizeof(Cmd));
	ZeroMemory(Read, sizeof(Read));

	bSMBusWrite(AM2320_ADDR, 0, 0, NULL);

	// wait?
	Cmd[0] = 0x00;
	Cmd[1] = 0x04;

	bOk = bSMBusWrite(AM2320_ADDR, 0x03, 2, Cmd);
	if (bOk != TRUE) return FALSE;

	bSMBusSequentialRead(AM2320_ADDR, 8, Read);
	uint16 crc, calc_crc;
	crc = (Read[7] << 8) | Read[6];
	calc_crc = CRC16Calc(Read, 6);
	if (crc != calc_crc) return FALSE;
	data->Humidity = (Read[2] << 8) | Read[3];
	data->Temp = (Read[4] << 8) | Read[5];


	return TRUE;
}

uint8 ToArray_AM2320(AM2320Data *data, uint8 *output, uint8 startidx) {
	uint8 start = startidx;
	output[startidx++] = data->Temp & 0x0F;
	output[startidx++] = data->Temp >> 8;
	output[startidx++] = data->Humidity & 0x0F;
	output[startidx++] = data->Humidity >> 8;
	// 差分=書き込んだ分
	return startidx - start;
}
