/**
 * @file i2c_handler.h
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#ifndef I2C_HANDLER_H_
#define I2C_HANDLER_H_

#include "em_i2c.h"
#define MMA8653FC_SDA_PORT      gpioPortA
#define MMA8653FC_SDA_PIN       3
#define MMA8653FC_SCL_PORT      gpioPortA
#define MMA8653FC_SCL_PIN       2

#define MMA8653FC_SCL_LOC       I2C_ROUTELOC0_SCLLOC_LOC1 // PA2
#define MMA8653FC_SDA_LOC       I2C_ROUTELOC0_SDALOC_LOC3 // PA3
// Public functions
void i2c_init(void);
void i2c_enable(void);
void i2c_disable(void);
void i2c_reset(void);
I2C_TransferSeq_TypeDef * i2c_transaction(I2C_TransferSeq_TypeDef * seq);

#endif // I2C_HANDLER_H_
