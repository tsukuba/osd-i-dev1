/*
 * PacketBuilder.c
 *
 *  Created on: 2015/08/14
 *      Author: Genya
 */

#include "PacketBuilder.h"


uint8 BuildPacket(uint16 opcode, uint8 *data, uint16 length, uint8 *out) {
	uint8 ptr = 0;
	uint16 i;

	// magic header
	out[ptr++] = 0x39;
	// op-code
	out[ptr++] = opcode & 0x0F;
	out[ptr++] = opcode >> 8;
	// length
	out[ptr++] = length & 0x0F;
	out[ptr++] = length >> 8;
	// data
	for (i = 0; i < length; i++) {
		out[ptr++] = data[i];
	}
	// crc
	uint16 crc = CRC16Calc(out, ptr);
	out[ptr++] = crc & 0xFF;
	out[ptr++] = crc >> 8;

	return ptr;
}
