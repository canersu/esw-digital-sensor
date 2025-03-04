/**
 * @file mma8653fc_driver.c
 *
 * @note    I2C set-up must be done separately and before the usage of this driver.
 *          GPIO interrupt set-up must be done separately if MMA8653FC interrupts are used.
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright ProLab, TTÜ. 2021
 */

#include "cmsis_os2.h" // For osDelay() in sensor_reset() function.
#include "mma8653fc_reg.h"
#include "mma8653fc_driver.h"
#include "em_i2c.h"
#include "i2c_handler.h"

#include "loglevels.h"
#define __MODUUL__ "sdrv"
#define __LOG_LEVEL__ (LOG_LEVEL_mmadrv & BASE_LOG_LEVEL)
#include "log.h"

static uint8_t read_registry(uint8_t regAddr);
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen);
static void write_registry(uint8_t regAddr, uint8_t regVal);
uint8_t read_whoami();


uint8_t read_whoami()
{
    return read_registry(MMA8653FC_REGADDR_WHO_AM_I);
}


/**
 * @brief   Reset MMA8653FC sensor (software reset).
 */
void sensor_reset (void)
{
    uint8_t regVal;
    
    regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG2);
    regVal = (regVal & ~MMA8653FC_CTRL_REG2_SOFTRST_MASK) | (MMA8653FC_CTRL_REG2_SOFTRST_EN << MMA8653FC_CTRL_REG2_SOFTRST_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG2, regVal);
    osDelay(5*osKernelGetTickFreq()/1000); // Wait a little for reset to finish.
}

/**
 * @brief   Sets sensor to active mode. 
 */
void set_sensor_active ()
{
    // Change mode to ACTIVE
    uint8_t reg_val;
    reg_val = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    reg_val = (reg_val & ~MMA8653FC_CTRL_REG1_SAMODE_MASK) | (MMA8653FC_CTRL_REG1_SAMODE_ACTIVE << MMA8653FC_CTRL_REG1_SAMODE_SHIFT);
    
    write_registry(MMA8653FC_REGADDR_CTRL_REG1, reg_val);
}

/**
 * @brief   Sets sensor to standby mode. Sensor must be in standby mode when writing to
 *          different config registries.
 */
void set_sensor_standby ()
{
    // Change mode to STANDBY
    uint8_t reg_val;
    reg_val = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    reg_val = (reg_val & ~MMA8653FC_CTRL_REG1_SAMODE_MASK) | (MMA8653FC_CTRL_REG1_SAMODE_STANDBY << MMA8653FC_CTRL_REG1_SAMODE_SHIFT);

    write_registry(MMA8653FC_REGADDR_CTRL_REG1, reg_val);
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   dataRate Set data rate (1.56, 6.26, 12, 50, 100, 200, 400, 800 Hz)
 * @param   range Set dynamic range (+- 2g, +- 4g, +- 8g)
 * @param   powerMod Set power mode (normal, low-noise-low-power, highres, low-power)
 * 
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_xyz_data (uint8_t dataRate, uint8_t range, uint8_t powerMod)
{
    // Check if sensor is in standby mode, control registers can only be modified in standby mode.
    uint8_t regVal, pow_regVal, cfg_regVal, dr_regVal;
    regVal = read_registry(MMA8653FC_REGADDR_SYSMOD);
    if (regVal != MMA8653FC_SYSMOD_MOD_STANDBY){return -1;}
    
    // Set data rate.
    dr_regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG1);
    dr_regVal = (dr_regVal & ~MMA8653FC_CTRL_REG1_DATA_RATE_MASK) | (dataRate << MMA8653FC_CTRL_REG1_DATA_RATE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG1, dr_regVal);
    
    // Set dynamic range.
    cfg_regVal = read_registry(MMA8653FC_REGADDR_XYZ_DATA_CFG);
    cfg_regVal = (cfg_regVal & ~MMA8653FC_XYZ_DATA_CFG_RANGE_MASK) | (range << MMA8653FC_XYZ_DATA_CFG_RANGE_SHIFT);
    write_registry(MMA8653FC_REGADDR_XYZ_DATA_CFG, cfg_regVal);
    
    // Set power mode (oversampling).
    pow_regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG2);
    pow_regVal = (pow_regVal & ~MMA8653FC_CTRL_REG2_ACTIVEPOW_MASK) | (powerMod << MMA8653FC_CTRL_REG2_ACTIVEPOW_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG2, pow_regVal);
    
    return 0;
}

/**
 * @brief   Configures MMA8653FC sensor to start collecting xyz acceleration data.
 *
 * @param   polarity Set interrupt pin polarity.
 * @param   pinmode Set interrupt pin pinmode.
 * @param   interrupt Set interrupts to use.
 * @param   int_select Route interrupts to selected pin.
 *
 * @return  -1 if sensor is not in standby mode
 *           0 if configuration succeeded (no check)
 */
int8_t configure_interrupt (uint8_t polarity, uint8_t pinmode, uint8_t interrupt, uint8_t int_select)
{
    // Check if sensor is in standby mode, control registers can only be modified in standby mode.
    uint8_t regVal, pol_regVal, dr_regVal, is_regVal;
    regVal = read_registry(MMA8653FC_REGADDR_SYSMOD);
    if (regVal != MMA8653FC_SYSMOD_MOD_STANDBY){return -1;}
    
    // Configure interrupt pin pinmode and interrupt transition direction
    pol_regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG3);
    pol_regVal = (pol_regVal & ~MMA8653FC_CTRL_REG3_POLARITY_MASK) | (polarity << MMA8653FC_CTRL_REG3_POLARITY_SHIFT);
    pol_regVal = (pol_regVal & ~MMA8653FC_CTRL_REG3_PINMODE_MASK) | (pinmode << MMA8653FC_CTRL_REG3_PINMODE_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG3, pol_regVal);

    // Enable data ready interrupt
    dr_regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG4);
    dr_regVal = (dr_regVal & ~MMA8653FC_CTRL_REG4_DRDY_INT_MASK) | (interrupt << MMA8653FC_CTRL_REG4_DRDY_INT_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG4, dr_regVal);


    // Route data ready interrupt to sensor INT1 output pin (connected to port PA1 on the TTTW uC)
    is_regVal = read_registry(MMA8653FC_REGADDR_CTRL_REG5);
    is_regVal = (is_regVal & ~MMA8653FC_CTRL_REG5_DRDY_INTSEL_MASK) | (int_select << MMA8653FC_CTRL_REG5_DRDY_INTSEL_SHIFT);
    write_registry(MMA8653FC_REGADDR_CTRL_REG5, is_regVal);

    return 0;
}

/**
 * @brief   Reads MMA8653FC STATUS and data registries.
 *
 * @return  Returns value of STATUS registry and x, y, z, 10 bit raw values (left-justified 2's complement)
 */
xyz_rawdata_t get_xyz_data()
{
    uint8_t rx_buf[7];
    xyz_rawdata_t data;

    read_multiple_registries(MMA8653FC_REGADDR_STATUS, rx_buf, 7);
    data.status = rx_buf[0];
    data.out_x = (uint16_t)(rx_buf[1] << 8) | (0x0000 | rx_buf[2]);
    data.out_y = (uint16_t)(rx_buf[3] << 8) | (0x0000 | rx_buf[4]);
    data.out_z = (uint16_t)(rx_buf[5] << 8) | (0x0000 | rx_buf[6]);
    
    return data;
}


/**
 * @brief   Read value of one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 *
 * @return  value of registry with address regAddr.
 */
static uint8_t read_registry(uint8_t regAddr)
{
    uint8_t reg;
    I2C_TransferSeq_TypeDef *ret, seq;
    static uint8_t tx_buf[1], rx_buf[1];
    
    // Configure I2C_TransferSeq_TypeDef
    seq.addr = MMA8653FC_SLAVE_ADDRESS_READ;
    tx_buf[0] = regAddr;
    seq.buf[0].data = tx_buf;
    seq.buf[0].len = 1;
    
    rx_buf[0] = 0;
    seq.buf[1].data = rx_buf;
    seq.buf[1].len = 1;
    seq.flags = I2C_FLAG_WRITE_READ;    
    
    // Read a value from MMA8653FC registry
    ret = i2c_transaction(&seq);
    reg = ret->buf[1].data[0];

    return reg;
}

/**
 * @brief   Write a value to one registry of MMA8653FC.
 *
 * @param   regAddr Address of registry to read.
 * @param   regVal Value to write to MMA8653FC registry.
 *
 * @note    rxBuf is not used I think, maybe replace with NULL pointer.
 */
static void write_registry(uint8_t regAddr, uint8_t regVal)
{
    uint8_t reg;
    I2C_TransferSeq_TypeDef *ret, seq;
    static uint8_t tx_buf[2], rx_buf[1];
    
    // Configure I2C_TransferSeq_TypeDef
    seq.addr = MMA8653FC_SLAVE_ADDRESS_WRITE;
    tx_buf[0] = regAddr;
    tx_buf[1] = regVal;
    seq.buf[0].data = tx_buf;
    seq.buf[0].len = 2;
    
    rx_buf[0] = 0;
    seq.buf[1].data = rx_buf;
    seq.buf[1].len = 1;
    seq.flags = I2C_FLAG_WRITE_WRITE;    
    
    // Read a value from MMA8653FC registry
    ret = i2c_transaction(&seq);
    
    return ;
}

/**
 * @brief   Read multiple registries of MMA8653FC in one go.
 * @note    MMA8653FC increments registry pointer to read internally according to its own logic. 
 *          Registries next to each other are not necessarily read in succession. Check MMA8653FC
 *          datasheet to see how registry pointer is incremented.
 *
 * @param   startRegAddr Address of registry to start reading from.
 * @param   *rxBuf Pointer to memory area where read values are stored.
 * @param   rxBufLen Length/size of rxBuf memory area.
 */
static void read_multiple_registries(uint8_t startRegAddr, uint8_t *rxBuf, uint16_t rxBufLen)
{
    uint8_t new_addr ,tmp, i;
    tmp = startRegAddr;
    for(i=0; i<rxBufLen; i++)
    {
        new_addr = read_registry(tmp);
        rxBuf[i] = new_addr;
        tmp+=1;
    }
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to decimal number representing MMA8653FC internal ADC read-out 
 *          (including bias).
 *          
 * @param raw_val   is expected to be left-justified 10-bit 2's complement number
 *
 * @return          decimal number ranging between -512 ... 511
 */
int16_t convert_to_count(uint16_t raw_val)
{
    signed int res;
    // Convert raw sensor data to ADC readout (count) value
    uint16_t shifted_val;
    // 10 bit sensor data shift operation
    shifted_val = raw_val >> 6;
    // Check sign bit
	if ((shifted_val & (0b1 << 9)) == 0) {
		// Return if value is positive
		res = shifted_val;
	}
	else {
		// Value is negative.
		unsigned char bit_idx = 0;
		unsigned int not_value = 0;
		for (bit_idx=0 ; bit_idx<=9 ; bit_idx++) {
			if ((shifted_val & (0b1 << bit_idx)) == 0) {
				not_value |= (0b1 << bit_idx);
			}
		}
		unsigned int absolute_value = not_value + 1;
		res = (-1) * absolute_value;
	}
    
    return res;
}

/**
 * @brief   Converts MMA8653FC sensor output value (left-justified 10-bit 2's complement
 *          number) to floating point number representing acceleration rate in g.
 *
 * @param raw_val       is expected to be left-justified 10-bit 2's complement number
 * @param sensor_scale  sensor scale 2g, 4g or 8g
 *
 * @return          floating point number, value depending on chosen sensor range 
 *                  +/- 2g  ->  range -2 ... 1.996
 *                  +/- 4g  ->  range -4 ... 3.992
 *                  +/- 8g  ->  range -8 ... 7.984
 */
float convert_to_g(uint16_t raw_val, uint8_t sensor_scale)
{
    // Convert raw sensor data to g-force acceleration value
    float res;
    uint8_t reg;

    reg = read_registry(MMA8653FC_REGADDR_XYZ_DATA_CFG);
    reg = (reg & ~MMA8653FC_XYZ_DATA_CFG_RANGE_MASK);

    res = convert_to_count(raw_val);

    if(reg == MMA8653FC_XYZ_DATA_CFG_2G_RANGE){res /= 256;}
    if(reg == MMA8653FC_XYZ_DATA_CFG_4G_RANGE){res /= 128;}
    if(reg == MMA8653FC_XYZ_DATA_CFG_8G_RANGE){res /= 64;}
    
    return res;
}
