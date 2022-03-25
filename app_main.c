/**
 * @file app_main.c
 * 
 * @brief   Communicates with TTTW labkit accelerometer over I2C protocol. Writes 
 *          results to log output.
 *
 * MMA8653FC datasheet
 * https://www.nxp.com/docs/en/data-sheet/MMA8653FC.pdf
 * 
 * MMA8653FC application note
 * https://www.nxp.com/docs/en/application-note/AN4083.pdf
 * 
 * EFR32 Application Note on I2C
 * https://www.silabs.com/documents/public/application-notes/AN0011.pdf
 *
 * EFR32MG12 Wireless Gecko Reference Manual (I2C p501)
 * https://www.silabs.com/documents/public/reference-manuals/efr32xg12-rm.pdf
 * 
 * EFR32MG12 Wireless Gecko datasheet
 * https://www.silabs.com/documents/public/data-sheets/efr32mg12-datasheet.pdf
 *
 * GPIO API documentation 
 * https://docs.silabs.com/mcu/latest/efr32mg12/group-GPIO
 * 
 * ARM RTOS API
 * https://arm-software.github.io/CMSIS_5/RTOS2/html/group__CMSIS__RTOS.html
 *
 * @author Johannes Ehala, ProLab.
 * @license MIT
 *
 * Copyright Thinnect Inc. 2019
 * Copyright ProLab, TTÜ. 2021
 * 
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "retargetserial.h"

#include "cmsis_os2.h"

#include "platform.h"

#include "SignatureArea.h"
#include "DeviceSignature.h"

#include "loggers_ext.h"
#include "logger_fwrite.h"

#include "i2c_handler.h"
#include "mma8653fc_reg.h"
#include "gpio_handler.h"
#include "mma8653fc_driver.h"
#include "app_main.h"

#include "loglevels.h"
#define __MODUUL__ "main"
#define __LOG_LEVEL__ (LOG_LEVEL_main & BASE_LOG_LEVEL)
#include "log.h"

// Include the information header binary
#include "incbin.h"
INCBIN(Header, "header.bin");

#define DATA_READY_FLAG     0x00000001U
static osThreadId_t dataReadyThreadId;

#ifdef CONVERT_TO_G
    static float buf1_x[ACC_XYZ_DATA_LEN];
    static float buf1_y[ACC_XYZ_DATA_LEN];
    static float buf1_z[ACC_XYZ_DATA_LEN];
#else
    static int16_t buf1_x[ACC_XYZ_DATA_LEN];
    static int16_t buf1_y[ACC_XYZ_DATA_LEN];
    static int16_t buf1_z[ACC_XYZ_DATA_LEN];
#endif


float calc_signal_energy(float buf[], uint32_t num_elements);

// Heartbeat loop - periodically print 'Heartbeat'
static void hb_loop (void *args)
{
    for (;;)
    {
        osDelay(10000);
        info1("Heartbeat");
    }
}

/**
 * @brief   Configures I2C, GPIO and sensor, wakes up on MMA8653FC data ready interrupt, fetches
 *          a batch of sensor data and analyzes data.
 */
static void mma_data_ready_loop (void *args)
{
    xyz_rawdata_t rawData;
    uint8_t res;
    static uint16_t buf_index = 0;
    float enX, enY, enZ;
    // Initialize and enable I2C.
    i2c_init();
    i2c_enable();

    // Read Who-am-I registry
    res = read_whoami();
    info1("Who-am-I %u %x", res, res);

    // To configure sensor put sensor in standby mode.
    set_sensor_standby();
    
    // Configure sensor for xyz data acquisition.
    res = configure_xyz_data(MMA8653FC_CTRL_REG1_DR_6HZ, SENSOR_DATA_RANGE, MMA8653FC_CTRL_REG2_POWMOD_LOWPOW);
    if(res != 0)debug1("Sensor conf failed");
    
    // Configure sensor to generate interrupt when new data becomes ready.
    res = configure_interrupt(INTERRUPT_POLARITY, INTERRUPT_PINMODE, INTERRUPT_DATA_READY, INTERRUPT_SELECTION);
    if(res != 0)debug1("Interrupt conf failed");
    
    // Configure GPIO for external interrupts and enable external interrupts.
    gpio_external_interrupt_enable(dataReadyThreadId, DATA_READY_FLAG);
    
    // Activate sensor.
    set_sensor_active();
    
    for (;;)
    {
        // Wait for data ready interrupt signal from MMA8653FC sensor
        osDelay(150*osKernelGetTickFreq()/1000);
        // Get raw data
        rawData = get_xyz_data();
        // info1("Status %02x, X: %04x Y: %04x Z: %04x", rawData.status, rawData.out_x, rawData.out_y, rawData.out_z);
        if(rawData.status == 15)
        {
            if(buf_index < ACC_XYZ_DATA_LEN)
            {
                // Convert to engineering value
                buf1_x[buf_index] = convert_to_g(rawData.out_x, SENSOR_DATA_RANGE);
                buf1_y[buf_index] = convert_to_g(rawData.out_y, SENSOR_DATA_RANGE);
                buf1_z[buf_index] = convert_to_g(rawData.out_z, SENSOR_DATA_RANGE);
                buf_index++;
                info1("Idx: %d X: %f Y: %f Z: %f", buf_index, buf1_x[buf_index] , buf1_y[buf_index], buf1_z[buf_index]);
            }
            else
            {
                // Signal analysis
                enX = calc_signal_energy(buf1_x, buf_index);
                enY = calc_signal_energy(buf1_y, buf_index);
                enZ = calc_signal_energy(buf1_z, buf_index);
                buf_index = 0;
                info2("Signal Energy");
                info2("x %i, %i", (int32_t)enX, abs((int32_t)(enX*1000) - (((int32_t)enX)*1000)));
                info2("y %i, %i", (int32_t)enY, abs((int32_t)(enY*1000) - (((int32_t)enY)*1000)));
                info2("z %i, %i", (int32_t)enZ, abs((int32_t)(enZ*1000) - (((int32_t)enZ)*1000)));
            }
        }
    }
}

int logger_fwrite_boot (const char *ptr, int len)
{
    fwrite(ptr, len, 1, stdout);
    fflush(stdout);
    return len;
}

int main ()
{
    PLATFORM_Init();

    // LEDs
    PLATFORM_LedsInit(); // This also enables GPIO peripheral.

    // Configure debug output.
    RETARGET_SerialInit();
    log_init(BASE_LOG_LEVEL, &logger_fwrite_boot, NULL);

    info1("Digi-sensor-demo "VERSION_STR" (%d.%d.%d)", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);

    // Initialize OS kernel.
    osKernelInitialize();

    // Create a thread.
    const osThreadAttr_t app_thread_attr = { .name = "heartbeat" , .priority = osPriorityNormal2 };
    osThreadNew(hb_loop, NULL, &app_thread_attr);

    // Create thread to receive data ready event and read data from sensor.
    const osThreadAttr_t data_ready_thread_attr = { .name = "data_ready_thread" };
    dataReadyThreadId = osThreadNew(mma_data_ready_loop, NULL, &data_ready_thread_attr);
    
    if (osKernelReady == osKernelGetState())
    {
        // Switch to a thread-safe logger
        logger_fwrite_init();
        log_init(BASE_LOG_LEVEL, &logger_fwrite, NULL);

        // Start the kernel
        osKernelStart();
    }
    else
    {
        err1("!osKernelReady");
    }

    for(;;);
}

/**
 * @brief 
 * Calculate energy of measured signal. 
 * 
 * @details 
 * Energy is calculated by subtracting bias from every sample and then adding 
 * together the square values of all samples. Energy is small if there is no 
 * signal (just measurement noise) and larger when a signal is present. 
 *
 * Disclaimer: The signal measured by the ADC is an elecrical signal, and its
 * unit would be joule, but since I don't know the exact load that the signal
 * is driving I can't account for the load. And so the energy I calculate here  
 * just indicates the presence or absence of a signal (and its relative 
 * strength), not the actual electrical energy in joules. 
 * Such a calculation can be done to all sorts of signals. There is probably
 * a more correct scientific term than energy for the result of this calculation
 * but I don't know what it is.
 *
 * Read about signal energy 
 * https://www.gaussianwaves.com/2013/12/power-and-energy-of-a-signal/
 *
 * @return Energy value.
 */
float calc_signal_energy(float buf[], uint32_t num_elements)
{
    static uint32_t i;
    static float signal_bias, signal_energy, res;

    signal_bias = signal_energy = res = 0;

    for (i = 0; i < num_elements; i++)
    {
        signal_bias += buf[i];
    }
    signal_bias /= num_elements;

    for (i = 0; i < num_elements; i++)
    {
        res = buf[i] - signal_bias; // Subtract bias
        signal_energy += res * res;
    }
    return signal_energy;
}
