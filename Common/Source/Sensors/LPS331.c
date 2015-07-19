#include "LPS331.h"

bool_t GetData_LPS331(LPS331Data *data) {
	bool_t bOk = TRUE;
	uint8 Cmd[8];
	uint8 Read[8];
	uint8 Press[3];
	uint8 Temp[2];

	ZeroMemory(Cmd, sizeof(Cmd));
	ZeroMemory(Read, sizeof(Read));
	ZeroMemory(Press, sizeof(Press));
	ZeroMemory(Temp, sizeof(Temp));

	bOk &= WriteI2CSingleByte(LPS331_ADDR, 0x20, 0x90);

	// WHO-AM-I
	//ReadI2CSingleByte(LPS331_ADDR, 0x20);
	bOk &= WriteI2CSingleByte(LPS331_ADDR, 0x20, 0x90);

	Press[0] = ReadI2CSingleByte(LPS331_ADDR, 0x28);
	Press[1] = ReadI2CSingleByte(LPS331_ADDR, 0x29);
	Press[2] = ReadI2CSingleByte(LPS331_ADDR, 0x2A);

	Temp[0] = ReadI2CSingleByte(LPS331_ADDR, 0x2B);
	Temp[0] = ReadI2CSingleByte(LPS331_ADDR, 0x2C);

	data->Pressure = (Press[2] << 16) | (Press[1] << 8) | Press[0];
	data->Temp = (Temp[1] << 8) | Temp[0];

	return bOk;
}

uint8 ToArray_LPS331(LPS331Data *data, uint8 *output, uint8 startidx) {
	return 0;
}
