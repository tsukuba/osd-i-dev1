/* BME280 Sensor */
#include "BME280.h"

BME280State state;
BME280RawData raw;
static int32 t_fine;

// Trim Data
static uint16 dig_T1;
static int16 dig_T2;
static int16 dig_T3;
static uint16 dig_P1;
static int16 dig_P2;
static int16 dig_P3;
static int16 dig_P4;
static int16 dig_P5;
static int16 dig_P6;
static int16 dig_P7;
static int16 dig_P8;
static int16 dig_P9;
static uint8 dig_H1;
static int16 dig_H2;
static uint8 dig_H3;
static int16 dig_H4;
static int16 dig_H5;
static int8 dig_H6;

// BME280の設定を行う
bool_t InitBME280() {
	bool_t bOk = TRUE;
	uint8 CTRL_MEAS_REG, CTRL_HUM_REG, CONFIG_REG;

	// 未セットアップ
	if (state != BME280_NOSETUP) state = BME280_NOSETUP;

	// Temperture oversampling | Pressure oversampling | Mode
	// x1 | x1 | NORMAL(3)
	CTRL_MEAS_REG = (1 << 5) | (1 << 2) | 3;
	// Humidity oversampling
	// x1
	CTRL_HUM_REG = 1;
	// Tstandby | Filter | 3-wire SPI
	// 1000ms | OFF | Disable
	CONFIG_REG = (5 << 5) | (0 << 2) | 0;

	// 設定を書き込む
	bOk &= WriteI2CSingleByte(BME280_ADDR, 0xF4, CTRL_MEAS_REG);
	bOk &= WriteI2CSingleByte(BME280_ADDR, 0xF2, CTRL_HUM_REG);
	bOk &= WriteI2CSingleByte(BME280_ADDR, 0xF5, CONFIG_REG);

	// Trimデータ未取得
	if (bOk == TRUE) state = BME280_NOTRIM;
	return bOk;
}

// BME280の補正データを取得する
bool_t GetTrimData_BME280() {
	bool_t bOk = TRUE;
	uint8 data[32];
	uint8 *p = data;

	// セットアップが終わっていないとTrimデータ取得しない
	// T-fineの状態であってもセットアップを要求する
	if (state != BME280_NOTRIM) return FALSE;

	ZeroMemory(data, sizeof(data));
	bOk &= ReadI2CMultiByte(BME280_ADDR, 0x88, 25, p);
	p += 25;
	bOk &= ReadI2CMultiByte(BME280_ADDR, 0xE1, 7, p);

	dig_T1 = data[1] << 8 | data[0];
	dig_T2 = data[3] << 8 | data[2];
	dig_T3 = data[5] << 8 | data[4];

	dig_P1 = data[7] << 8 | data[6];
	dig_P2 = data[9] << 8 | data[8];
	dig_P3 = data[11] << 8 | data[10];
	dig_P4 = data[13] << 8 | data[12];
	dig_P5 = data[15] << 8 | data[14];
	dig_P6 = data[17] << 8 | data[16];
	dig_P7 = data[19] << 8 | data[18];
	dig_P8 = data[21] << 8 | data[20];
	dig_P9 = data[23] << 8 | data[22];

	dig_H1 = data[24];
	dig_H2 = data[26] << 8 | data[25];
	dig_H3 = data[27];
	dig_H4 = data[28] << 4 | (data[29] & 0x0F);
	dig_H5 = data[30] << 4 | ((data[29] & 0x0F) >> 4);
	dig_H6 = data[31];

	if (bOk == TRUE) state = BME280_NODATA;
	return bOk;
}

// センサの生データを取得する
bool_t GetADC_BME280() {
	bool_t bOk = TRUE;
	uint8 data[8];

	if (state != BME280_NODATA) return FALSE;
	bOk = ReadI2CMultiByte(BME280_ADDR, 0xF7, 8, data);

	raw.Press_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
	raw.Temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
	raw.Hum_raw = (data[6] << 8) | data[7];

	if (bOk == TRUE) state = BME280_NOFINE;
	return bOk;
}

// t-fine(定数)を計算する
bool_t CalcTfine_BME280() {
	int32 var1, var2;

	// Trim情報が無い場合t-fineを求めない
	// t-fineの再計算も受け付けない
	if (state != BME280_NOFINE) return FALSE;

	var1 = ((((raw.Temp_raw >> 3) - ((int32)dig_T1 << 1))) * ((int32)dig_T2)) >> 11;
	var2 = (((((raw.Temp_raw >> 4) - ((int32)dig_T1)) * ((raw.Temp_raw >> 4) - ((int32)dig_T1))) >> 12) * ((int32)dig_T3)) >> 14;

	t_fine = var1 + var2;

	state = BME280_READY;

	return TRUE;
}

// 温度を計算する
int32 GetTemp_BME280() {
	// 変なステートで計算しようとしたらエラーとする
	if (state != BME280_READY) return 0;
	return (t_fine * 5 + 128) >> 8;
}

// 気圧を計算する
uint32 GetPressure_BME280() {
	int32 var1, var2;
	uint32 P;
	// 変なステートで計算しようとしたらエラーとする
	if (state != BME280_READY) return 0;

	var1 = (((int32)t_fine) >> 1) - (int32)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32)dig_P6);
	var2 += ((var1 * ((int32)dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32)dig_P4) << 16);
	var1 = (((dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32)dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((var1 + 32768)) * ((int32)dig_P1)) >> 15);
	if (var1 == 0) {
		return 0;
	}
	P = (((uint32)(((int32)1048576) - raw.Press_raw) - (var2 >> 12))) * 3125;
	if (P < 0x80000000) {
		P = (P << 1) / ((uint32)var1);
	} else {
		P = (P / (uint32)var1) * 2;
	}
	var1 = (((int32)dig_P9) * ((int32)(((P >> 3) * (P >> 3)) >> 13))) >> 12;
	var2 = (((int32)(P >> 2)) * ((int32)dig_P8)) >> 13;
	P = (uint32)((int32)P + ((var1 + var2 + dig_P7) >> 4));
	return P;
}

uint32 GetHumidity_BME280() {
	int32 v_x1;
	// 変なステートで計算しようとしたらエラーとする
	if (state != BME280_READY) return 0;

	v_x1 = (t_fine - ((int32)76800));
	v_x1 = (((((raw.Hum_raw << 14> - (((int32)dig_H4) << 20) - (((int32)dig_H5) * v_x1)) + ((int32)16384)) >> 15) *
			(((((((v_x1 * ((int32)dig_H6)) >> 10) * (((v_x1 * (int32)dig_H3)) >> 11) + ((int32)32768))) >> 10) + ((int32)2097152)) * ((int32)dig_H2) + 8192) >> 14));
	v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((int32)dig_H1)) >> 4));
	v_x1 = (v_x1 < 0 ? 0 : v_x1);
	v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
	return (uint32)(v_x1 >> 12);
}

bool_t GetData_BME280(BME280Data *data) {
	// init
	ZeroMemory(&raw, sizeof(raw));
	state = BME280_NOSETUP;

	if (InitBME280() == FALSE) return FALSE;
	if (GetTrimData_BME280() == FALSE) return FALSE;
	if (GetADC_BME280() == FALSE) return FALSE;
	if (CalcTfine_BME280() == FALSE) return FALSE;

	// 準備が完了したので計算する
	if (state == BME280_READY) {
		data->Humidity = GetHumidity_BME280();
		data->Temp = GetTemp_BME280();
		data->Pressure = GetPressure_BME280();
	}

	return TRUE;
}

uint8 ToArray_BME280(BME280Data *data, uint8 *output, uint8 startidx) {
	uint8 start = startidx;
	// Temp
	output[start++] = data->Temp & 0xFF;
	output[start++] = data->Temp >> 8;
	output[start++] = data->Temp >> 16;
	output[start++] = data->Temp >> 24;
	// Humidity
	output[start++] = data->Humidity & 0xFF;
	output[start++] = data->Humidity >> 8;
	output[start++] = data->Humidity >> 16;
	output[start++] = data->Humidity >> 24;
	// Pressure
	output[start++] = data->Pressure & 0xFF;
	output[start++] = data->Pressure >> 8;
	output[start++] = data->Pressure >> 16;
	output[start++] = data->Pressure >> 24;
	// 差分=書き込んだ分
	return startidx - start;
}
