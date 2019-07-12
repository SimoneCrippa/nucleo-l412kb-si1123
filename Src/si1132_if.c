/*
 * si1132_if.c
 *
 *  Created on: Jul 9, 2019
 *      Author: Giuseppe Massari
 */

#include "si1132_if.h"

#define SI1132_WAIT_DELAY 10

HAL_StatusTypeDef Si1132_Reset(I2C_HandleTypeDef * h_i2c)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t val[] = { Si1132_REG_MEASRATE0, 0};
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);
	HAL_Delay(SI1132_WAIT_DELAY);

	val[0] = Si1132_REG_MEASRATE1;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_IRQEN;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_IRQMODE1;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_IRQMODE2;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_INTCFG;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_IRQSTAT;
	val[1] = 0xFF;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_RESET;
	val[1] = 1;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_HWKEY;
	val[1] = 0x17;
	status = HAL_I2C_Master_Transmit(h_i2c,
			Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);
	return status;
}

void Si1132_Init(I2C_HandleTypeDef * h_i2c)
{
	HAL_StatusTypeDef status = HAL_OK;

	while(Si1132_Reset(h_i2c) != HAL_OK);

	uint8_t val[] = { Si1132_REG_UCOEF0, 0x7B};
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_UCOEF1;
	val[1] = 0x6B;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_UCOEF2;
	val[1] = 0x01;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_UCOEF3;
	val[1] = 0x00;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_CHLIST, Si1132_PARAM_CHLIST_ENUV |
			Si1132_PARAM_CHLIST_ENALSIR | Si1132_PARAM_CHLIST_ENALSVIS);

	val[0] = Si1132_REG_INTCFG;
	val[1] = Si1132_REG_INTCFG_INTOE;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_IRQEN;
	val[1] = Si1132_REG_IRQEN_ALSEVERYSAMPLE;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_ALSIRADCMUX, Si1132_PARAM_ADCMUX_SMALLIR);

	// fastest clocks, clock div 1
	Si1132_I2C_writeParam(h_i2c, Si1132_PARAM_ALSIRADCGAIN, 0);
	// take 511 clocks to measure
	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_ALSIRADCCOUNTER, Si1132_PARAM_ADCCOUNTER_511CLK);
	// in high range mode
	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_ALSIRADCMISC, Si1132_PARAM_ALSIRADCMISC_RANGE);
	// fastest clocks
	Si1132_I2C_writeParam(h_i2c, Si1132_PARAM_ALSVISADCGAIN, 0);
	// take 511 clocks to measure
	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_ALSVISADCCOUNTER, Si1132_PARAM_ADCCOUNTER_511CLK);
	//in high range mode (not normal signal)
	Si1132_I2C_writeParam(h_i2c,
			Si1132_PARAM_ALSVISADCMISC, Si1132_PARAM_ALSVISADCMISC_VISRANGE);

	val[0] = Si1132_REG_MEASRATE0;
	val[1] = 0xFF;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_COMMAND;
	val[1] = Si1132_ALS_AUTO;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);
}


void Si1132_I2C_writeParam(
		I2C_HandleTypeDef * h_i2c, uint8_t param, uint8_t value)
{
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t val[] = { Si1132_REG_PARAMWR, value };
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);

	val[0] = Si1132_REG_COMMAND;
	val[1] = param | Si1132_PARAM_SET;
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 2, HAL_MAX_DELAY);
}

uint8_t Si1132_I2C_read8(
		I2C_HandleTypeDef * h_i2c, uint8_t reg)
{
	uint8_t ret;
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t val[] = { reg };
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 1, HAL_MAX_DELAY);
	status = HAL_I2C_Master_Receive(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) &ret, 1, HAL_MAX_DELAY);
	return ret;
}

uint16_t Si1132_I2C_read16(
		I2C_HandleTypeDef * h_i2c, uint8_t reg)
{
	uint8_t rbuf[2];
	HAL_StatusTypeDef status = HAL_OK;

	uint8_t val[] = { reg };
	status = HAL_I2C_Master_Transmit(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) val, 1, HAL_MAX_DELAY);
	status = HAL_I2C_Master_Receive(h_i2c,
				Si1132_DEVICE_ADDR, (uint8_t *) rbuf, sizeof(rbuf), HAL_MAX_DELAY);

	uint16_t retval = rbuf[1];
	return (uint16_t)(rbuf[0] | retval << 8);
}


void Si1132_readVisible(I2C_HandleTypeDef * h_i2c, float * value)
{
	uint16_t rval = Si1132_I2C_read16(h_i2c, 0x22);
	*value = (float) ((rval - 256)/0.282)*14.5;
}

void Si1132_readIR(I2C_HandleTypeDef * h_i2c, float * value)
{
	uint16_t rval = Si1132_I2C_read16(h_i2c, 0x24);
	*value = (float) ((rval - 250)/2.44)*14.5;
}

uint16_t Si1132_readUV(I2C_HandleTypeDef * h_i2c)
{
	return Si1132_I2C_read16(h_i2c, 0x2c);
}

