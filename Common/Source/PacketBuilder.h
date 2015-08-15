/* PacketBuilder Header */
#ifndef PACKETBUILDER_H_
#define PACKETBUILDER_H_
#include <jendefs.h>
#include "SensorUtil.h"
uint8 BuildPacket(uint16 opcode, uint8 *data, uint16 length, uint8 *out);
#endif /* PACKETBUILDER_H_ */
