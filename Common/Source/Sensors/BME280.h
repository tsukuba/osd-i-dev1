/* BME280 Sensor */
#include "SensorUtil.h"

#define BME280_ADDR (0x76)

typedef struct {
	int32 Temp;
	uint32 Humidity;
	uint32 Pressure;
} BME280Data;

typedef struct {
	uint32 Temp_raw;
	uint32 Press_raw;
	uint32 Hum_raw;
} BME280RawData;

typedef enum {
	BME280_NOSETUP = 1,
	BME280_NOTRIM = 2,
	BME280_NODATA = 3,
	BME280_NOFINE = 4,
	BME280_READY = 5
} BME280State;

// Exportする分
bool_t GetData_BME280(BME280Data *data);
uint8 ToArray_BME2800(BME280Data *data, uint8 *output, uint8 startidx);
