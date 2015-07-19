/* LPS331 Sensor */
#include "SensorUtil.h"

#define LPS331_ADDR (0x5D)

typedef struct {
	int32 Temp;
	uint32 Pressure;
} LPS331Data;

bool_t GetData_LPS331(LPS331Data *data);
uint8 ToArray_LPS331(LPS331Data *data, uint8 *output, uint8 startidx);
