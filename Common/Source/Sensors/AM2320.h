/* AM2320 Header */
#ifndef AM2320_H_
#define AM2320_H_
#include "SensorUtil.h"

#define AM2320_ADDR (0x5C)

typedef struct {
	uint16 Temp;
	uint16 Humidity;
} AM2320Data;

bool_t GetData_AM2320(AM2320Data *data);
uint8 ToArray_AM2320(AM2320Data *data, uint8 *output, uint8 startidx);
#endif /* AM2320_H_ */
