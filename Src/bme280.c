// Functions to manage the BME280 sensor:
//   - get version
//   - get chip ID
//   - reset the chip
//   - read calibration parameters
//   - configure the chip
//   - read uncompensated values of temperature, pressure and humidity
//   - calculate compensated values of temperature, pressure and humidity
//   - calculate pressure in millimeters of mercury
//   - calculate barometric altitude


#include "bme280.h"


// Carries fine temperature as global value for pressure and humidity calculation
static int32_t t_fine;


// Write new value to BME280 register
// input:
//   reg - register number
//   value - new register value
void BME280_WriteReg(I2C_HandleTypeDef * h_i2c, uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };

	HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,buf,2,HAL_MAX_DELAY);
}

// Read BME280 register
// input:
//   reg - register number
// return:
//   register value
uint8_t BME280_ReadReg(I2C_HandleTypeDef * h_i2c,uint8_t reg) {
	uint8_t value = 0; // Initialize value in case of I2C timeout

	// Send register address
	HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&reg,1,HAL_MAX_DELAY);
	// Read register value
	HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,&value,1,HAL_MAX_DELAY);

	return value;
}

// Check if BME280 present on I2C bus
// return:
//   BME280_SUCCESS if BME280 present, BME280_ERROR otherwise (not present or it was an I2C timeout)
BME280_RESULT BME280_Check(I2C_HandleTypeDef * h_i2c) {
	return (BME280_ReadReg(h_i2c,BME280_REG_ID) == 0x60) ? BME280_SUCCESS : BME280_ERROR;
}

// Order BME280 to do a software reset
// note: after reset the chip will be unaccessible during 3ms
void BME280_Reset(I2C_HandleTypeDef * h_i2c) {
	BME280_WriteReg(h_i2c,BME280_REG_RESET,BME280_SOFT_RESET_KEY);
}

// Get version of the BME280 chip
// return:
//   BME280 chip version or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t BME280_GetVersion(I2C_HandleTypeDef * h_i2c) {
	return BME280_ReadReg(h_i2c, BME280_REG_ID);
}

// Get current status of the BME280 chip
// return:
//   Status of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t BME280_GetStatus(I2C_HandleTypeDef * h_i2c) {
	return BME280_ReadReg(h_i2c, BME280_REG_STATUS) & BME280_STATUS_MSK;
}

// Get current sensor mode of the BME280 chip
// return:
//   Sensor mode of the BME280 chip or zero if no BME280 present on the I2C bus or it was an I2C timeout
uint8_t BME280_GetMode(I2C_HandleTypeDef * h_i2c) {
	return BME280_ReadReg(h_i2c, BME280_REG_CTRL_MEAS) & BME280_MODE_MSK;
}

// Set sensor mode of the BME280 chip
// input:
//   mode - new mode (BME280_MODE_SLEEP, BME280_MODE_FORCED or BME280_MODE_NORMAL)
void BME280_SetMode(I2C_HandleTypeDef * h_i2c,uint8_t mode) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'mode' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CTRL_MEAS) & ~BME280_MODE_MSK;

	// Configure new mode
	reg |= mode & BME280_MODE_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CTRL_MEAS,reg);
}

// Set coefficient of the IIR filter
// input:
//   filter - new coefficient value (one of BME280_FILTER_x values)
void BME280_SetFilter(I2C_HandleTypeDef * h_i2c,uint8_t filter) {
	uint8_t reg;

	// Read the 'config' (0xF5) register and clear 'filter' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CONFIG) & ~BME280_FILTER_MSK;

	// Configure new filter value
	reg |= filter & BME280_FILTER_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CONFIG,reg);
}

// Set inactive duration in normal mode (Tstandby)
// input:
//   tsb - new inactive duration (one of BME280_STBY_x values)
void BME280_SetStandby(I2C_HandleTypeDef * h_i2c, uint8_t tsb) {
	uint8_t reg;

	// Read the 'config' (0xF5) register and clear 'filter' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CONFIG) & ~BME280_STBY_MSK;

	// Configure new standby value
	reg |= tsb & BME280_STBY_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CONFIG,reg);
}

// Set oversampling of temperature data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_T_Xx values)
void BME280_SetOSRST(I2C_HandleTypeDef * h_i2c, uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_t' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CTRL_MEAS) & ~BME280_OSRS_T_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_T_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CTRL_MEAS,reg);
}

// Set oversampling of pressure data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_P_Xx values)
void BME280_SetOSRSP(I2C_HandleTypeDef * h_i2c, uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_meas' (0xF4) register and clear 'osrs_p' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CTRL_MEAS) & ~BME280_OSRS_P_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_P_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CTRL_MEAS,reg);
}

// Set oversampling of humidity data
// input:
//   osrs - new oversampling value (one of BME280_OSRS_H_Xx values)
void BME280_SetOSRSH(I2C_HandleTypeDef * h_i2c, uint8_t osrs) {
	uint8_t reg;

	// Read the 'ctrl_hum' (0xF2) register and clear 'osrs_h' bits
	reg = BME280_ReadReg(h_i2c, BME280_REG_CTRL_HUM) & ~BME280_OSRS_H_MSK;

	// Configure new oversampling value
	reg |= osrs & BME280_OSRS_H_MSK;

	// Write value back to the register
	BME280_WriteReg(h_i2c, BME280_REG_CTRL_HUM,reg);

	// Changes to 'ctrl_hum' register only become effective after a write to 'ctrl_meas' register
	// Thus read a value of the 'ctrl_meas' register and write it back after write to the 'ctrl_hum'

	// Read the 'ctrl_meas' (0xF4) register
	reg = BME280_ReadReg(h_i2c, BME280_REG_CTRL_MEAS);

	// Write back value of 'ctrl_meas' register to activate changes in 'ctrl_hum' register
	BME280_WriteReg(h_i2c, BME280_REG_CTRL_MEAS,reg);
}

// Read calibration data
BME280_RESULT BME280_Read_Calibration(I2C_HandleTypeDef * h_i2c) {
	uint8_t buf[7];

	// Read pressure and temperature calibration data (calib00..calib23)
	buf[0] = BME280_REG_CALIB00; // calib00 register address
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,(uint8_t *)&cal_param,24,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;

	// Skip one byte (calib24) and read H1 (calib25)
	cal_param.dig_H1 = BME280_ReadReg(h_i2c,BME280_REG_CALIB25);

	// Read humidity calibration data (calib26..calib41)
	buf[0] = BME280_REG_CALIB26; // calib26 register address
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,buf,7,HAL_MAX_DELAY != HAL_OK)) return BME280_ERROR;

	// Unpack data
	cal_param.dig_H2 = (int16_t)((((int8_t)buf[1]) << 8) | buf[0]);
	cal_param.dig_H3 = buf[2];
	cal_param.dig_H4 = (int16_t)((((int8_t)buf[3]) << 4) | (buf[4] & 0x0f));
	cal_param.dig_H5 = (int16_t)((((int8_t)buf[5]) << 4) | (buf[4]  >>  4));
	cal_param.dig_H6 = (int8_t)buf[6];

	return BME280_SUCCESS;
}

// Read the raw pressure value
// input:
//   UP = pointer to store value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: '0x80000' result means no data for pressure (measurement skipped or not ready yet)
BME280_RESULT BME280_Read_UP(I2C_HandleTypeDef * h_i2c,int32_t *UP) {
	uint8_t buf[3];

	// Clear result value
	*UP = 0x80000;

	// Send 'press_msb' register address
	buf[0] = BME280_REG_PRESS_MSB;
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;

	// Read the 'press' register (_msb, _lsb, _xlsb)
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,&buf[0],3,HAL_MAX_DELAY) == HAL_OK) {
		*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));

		return BME280_SUCCESS;
	}

	return BME280_ERROR;
}

// Read the raw temperature value
// input:
//   UT = pointer to store value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: '0x80000' result means no data for temperature (measurement skipped or not ready yet)
BME280_RESULT BME280_Read_UT(I2C_HandleTypeDef * h_i2c, int32_t *UT) {
	uint8_t buf[3];

	// Clear result value
	*UT = 0x80000;

	// Send 'temp_msb' register address
	buf[0] = BME280_REG_TEMP_MSB;
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;

	// Read the 'temp' register (_msb, _lsb, _xlsb)
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,&buf[0],3,HAL_MAX_DELAY) == HAL_OK) {
		*UT = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));

		return BME280_SUCCESS;
	}

	return BME280_ERROR;
}


// Read the raw humidity value
// input:
//   UH = pointer to store value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: '0x8000' result means no data for humidity (measurement skipped or not ready yet)
BME280_RESULT BME280_Read_UH(I2C_HandleTypeDef * h_i2c,int32_t *UH) {
	uint8_t buf[2];

	// Clear result value
	*UH = 0x8000;

	// Send 'hum_msb' register address
	buf[0] = BME280_REG_HUM_MSB;
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;

	// Read the 'hum' register (_msb, _lsb)
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,&buf[0],2,BME280_ADDR) == HAL_OK) {
		*UH = (int32_t)((buf[0] << 8) | buf[1]);

		return BME280_SUCCESS;
	}

	return BME280_ERROR;
}

// Read all raw values
// input:
//   UT = pointer to store temperature value
//   UP = pointer to store pressure value
//   UH = pointer to store humidity value
// return:
//   BME280_ERROR in case of I2C timeout, BME280_SUCCESS otherwise
// note: 0x80000 value for UT and UP and 0x8000 for UH means no data
BME280_RESULT BME280_Read_UTPH(I2C_HandleTypeDef * h_i2c,int32_t *UT, int32_t *UP, int32_t *UH) {
	uint8_t buf[8];

	// Clear result values
	*UT = 0x80000;
	*UP = 0x80000;
	*UH = 0x8000;

	// Send 'press_msb' register address
	buf[0] = BME280_REG_PRESS_MSB;
	if (HAL_I2C_Master_Transmit(h_i2c,BME280_I2C_PORT,&buf[0],1,HAL_MAX_DELAY) != HAL_OK) return BME280_ERROR;

	// Read the 'press', 'temp' and 'hum' registers
	if (HAL_I2C_Master_Receive(h_i2c,BME280_I2C_PORT,&buf[0],8,HAL_MAX_DELAY) == HAL_OK) {
		*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		*UT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
		*UH = (int32_t)((buf[6] <<  8) |  buf[7]);

		return BME280_SUCCESS;
	}

	return BME280_ERROR;
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
int32_t BME280_CalcT(int32_t UT) {
	t_fine  = ((((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1))) * ((int32_t)cal_param.dig_T2)) >> 11;
	t_fine += (((((UT >> 4) - ((int32_t)cal_param.dig_T1)) * ((UT >> 4) - ((int32_t)cal_param.dig_T1))) >> 12) * ((int32_t)cal_param.dig_T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
}

// Calculate pressure from raw value, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa as unsigned 32-bit integer in Q24.8 format (24 integer and 8 fractional bits)
// note: output value of '24674867' represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// note: BME280_CalcT must be called before calling this function
// note: using 64-bit calculations
// note: code from the BME280 datasheet (rev 1.1)
uint32_t BME280_CalcP(int32_t UP) {
#if (BME280_USE_INT64)
	int64_t v1,v2,p;

	v1 = (int64_t)t_fine - 128000;
	v2 = v1 * v1 * (int64_t)cal_param.dig_P6;
	v2 = v2 + ((v1 * (int64_t)cal_param.dig_P5) << 17);
	v2 = v2 + ((int64_t)cal_param.dig_P4 << 35);
	v1 = ((v1 * v1 * (int64_t)cal_param.dig_P3) >> 8) + ((v1 * (int64_t)cal_param.dig_P2) << 12);
	v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)cal_param.dig_P1) >> 33;
	if (v1 == 0) return 0; // avoid exception caused by division by zero
	p = 1048576 - UP;
	p = (((p << 31) - v2) * 3125) / v1;
	v1 = (((int64_t)cal_param.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	v2 = (((int64_t)cal_param.dig_P8) * p) >> 19;
	p = ((p + v1 + v2) >> 8) + ((int64_t)cal_param.dig_P7 << 4);
#else // BME280_USE_INT64
	int32_t v1,v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)cal_param.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)cal_param.dig_P4) << 16);
	v1 = (((cal_param.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13 )) >> 3) + ((((int32_t)cal_param.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)cal_param.dig_P1)) >> 15;
	if (v1 == 0) return 0; // avoid exception caused by division by zero
	p = (((uint32_t)(((int32_t)1048576) - UP) - (v2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)v1);
	} else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)cal_param.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + cal_param.dig_P7) >> 4));

	// Convert pressure to Q24.8 format (fractional part always be .000)
	p <<= 8;
#endif // BME280_USE_INT64

	return (uint32_t)p;
}

// Calculate humidity from raw value, resolution is 0.001 %RH
// input:
//   UH - raw humidity value
// return: humidity in %RH as unsigned 32-bit integer in Q22.10 format (22 integer and 10 fractional bits)
// note: output value of '47445' represents 47445/1024 = 46.333 %RH
// note: BME280_CalcT must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
uint32_t BME280_CalcH(int32_t UH) {
	int32_t vx1;

	vx1  = t_fine - (int32_t)76800;
	vx1  = ((((UH << 14) - ((int32_t)cal_param.dig_H4 << 20) - ((int32_t)cal_param.dig_H5 * vx1)) +	(int32_t)16384) >> 15) *
			(((((((vx1 * (int32_t)cal_param.dig_H6) >> 10) * (((vx1 * (int32_t)cal_param.dig_H3) >> 11) +
			(int32_t)32768)) >> 10) + (int32_t)2097152) * ((int32_t)cal_param.dig_H2) + 8192) >> 14);
	vx1 -= ((((vx1 >> 15) * (vx1 >> 15)) >> 7) * (int32_t)cal_param.dig_H1) >> 4;
	vx1  = (vx1 < 0) ? 0 : vx1;
	vx1  = (vx1 > 419430400) ? 419430400 : vx1;

	return (uint32_t)(vx1 >> 12);
}


#if (BME280_USE_FLOAT)
// Calculate temperature from raw value using floats, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '51.23' equals '51.23C')
// note: code from the BME280 datasheet (rev 1.1)
float BME280_CalcTf(int32_t UT) {
	float v_x1,v_x2;

	v_x1 = (((float)UT) / 16384.0 - ((float)cal_param.dig_T1) / 1024.0) * ((float)cal_param.dig_T2);
	v_x2 = ((float)UT) / 131072.0 - ((float)cal_param.dig_T1) / 8192.0;
	v_x2 = (v_x2 * v_x2) * ((float)cal_param.dig_T3);
	t_fine = (uint32_t)(v_x1 + v_x2);


	return ((v_x1 + v_x2) / 5120.0);
}

// Calculate pressure from raw value using floats, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in Pa (value of '99158.968' represents 99158.968Pa)
// note: BME280_CalcT of BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
float BME280_CalcPf(uint32_t UP) {
	float v_x1, v_x2, p;

	v_x1 = ((float)t_fine / 2.0) - 64000.0;
	v_x2 = v_x1 * v_x1 * ((float)cal_param.dig_P6) / 32768.0;
	v_x2 = v_x2 + v_x1 * ((float)cal_param.dig_P5) * 2.0;
	v_x2 = (v_x2 / 4.0) + (((float)cal_param.dig_P4) * 65536.0);
	v_x1 = (((float)cal_param.dig_P3) * v_x1 * v_x1 / 524288.0 + ((float)cal_param.dig_P2) * v_x1) / 524288.0;
	v_x1 = (1.0 + v_x1 / 32768.0) * ((float)cal_param.dig_P1);
	p = 1048576.0 - (float)UP;
	if (v_x1 == 0) return 0; // Avoid exception caused by division by zero
	p = (p - (v_x2 / 4096.0)) * 6250.0 / v_x1;
	v_x1 = ((float)cal_param.dig_P9) * p * p / 2147483648.0;
	v_x2 = p * ((float)cal_param.dig_P8) / 32768.0;
	p += (v_x1 + v_x2 + ((float)cal_param.dig_P7)) / 16.0;

	return p;
}

// Calculate humidity from raw value using floats, resolution is 0.001 %RH
// input:
//   UH - raw humidity value
// return: humidity in %RH (value of '46.333' represents 46.333%RH)
// note: BME280_CalcT or BME280_CalcTf must be called before calling this function
// note: code from the BME280 datasheet (rev 1.1)
float BME280_CalcHf(uint32_t UH) {
	float h;

	h = (((float)t_fine) - 76800.0);
	if (h == 0) return 0;
	h = (UH - (((float)cal_param.dig_H4) * 64.0 + ((float)cal_param.dig_H5) / 16384.0 * h));
	h = h * (((float)cal_param.dig_H2) / 65536.0 * (1.0 + ((float)cal_param.dig_H6) / 67108864.0 * h * (1.0 + ((float)cal_param.dig_H3) / 67108864.0 * h)));
	h = h * (1.0 - ((float)cal_param.dig_H1) * h / 524288.0);
	if (h > 100.0) {
		h = 100.0;
	} else if (h < 0.0) {
		h = 0.0;
	}

	return h;
}
#endif // BME280_USE_FLOAT
