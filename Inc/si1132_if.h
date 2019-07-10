/*
 * si1132_if.h

 *
 *  Created on: Jul 9, 2019
 *      Author: Giuseppe Massari
 */

#ifndef SI1132_IF_H_
#define SI1132_IF_H_

#include "si1132_defs.h"
#include "stm32l4xx_hal.h"


void Si1132_Reset(I2C_HandleTypeDef *);

void Si1132_Init(I2C_HandleTypeDef *);

void Si1132_I2C_writeParam(I2C_HandleTypeDef *, uint8_t, uint8_t);

void Si1132_readVisible(I2C_HandleTypeDef * h_i2c, float * value);

void Si1132_readIR(I2C_HandleTypeDef * h_i2c, float * value);

uint16_t Si1132_readUV(I2C_HandleTypeDef * h_i2c);


#endif /* SI1132_IF_H_ */
