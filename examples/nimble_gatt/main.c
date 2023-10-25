/*
 * Copyright (C) 2018 Freie Universität Berlin
 *               2018 Codecoup
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       BLE peripheral example using NimBLE
 *
 * Have a more detailed look at the api here:
 * https://mynewt.apache.org/latest/tutorials/ble/bleprph/bleprph.html
 *
 * More examples (not ready to run on RIOT) can be found here:
 * https://github.com/apache/mynewt-nimble/tree/master/apps
 *
 * Test this application e.g. with Nordics "nRF Connect"-App
 * iOS: https://itunes.apple.com/us/app/nrf-connect/id1054362403
 * Android: https://play.google.com/store/apps/details?id=no.nordicsemi.android.mcp
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Andrzej Kaczmarek <andrzej.kaczmarek@codecoup.pl>
 * @author      Hendrik van Essen <hendrik.ve@fu-berlin.de>
 *
 * @}
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "nimble_riot.h"
#include "nimble_autoadv.h"

#include "thread.h"
#include "shell.h"
#include "ztimer.h"
#include "xtimer.h"

#include "periph/i2c.h"

#include "bmi160.h"
#define BMM150_USE_FLOATING_POINT 
#include "bmm150.h"
#include "bmx280_params.h"
#include "bmx280.h"

#include <stdint.h>
#include "event/timeout.h"
// #include "net/bluetil/ad.h"
#include "timex.h"

/* Macros for frames to be read */

#define MAG_FRAMES 10 /* 10 Frames are available every 25ms @ 400 Hz */
/* 10 frames containing a 1 byte header and 8 bytes of magnetometer data,
 * This results in 9 bytes per frame*/
#define FIFO_SIZE 90

/* Variable declarations */
struct bmi160_dev bmi;
struct bmm150_dev bmm;

struct bmm150_settings bmm_settings;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_aux_data aux_data[MAG_FRAMES];
struct bmm150_mag_data mag_data[MAG_FRAMES];

int8_t rslt;

typedef struct
{
    float X_axis;
    float Y_axis;
    float Z_axis;
} leitura;

#define MAX_READINGS 200

leitura readings_buffer[MAX_READINGS];
int write_index;

void init_bmi_bmm_Sensors(void);
void direct_read(void);
void acquire_MAG_Values(void);
void acquire_read(void);


int dev = I2C_DEV(0);

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_read_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_read_regs(dev, dev_addr, reg_addr, data, len, 0);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    // printf("i2c_write_regs(%d, %x, %x, ...)\n", dev, dev_addr, reg_addr);
    return i2c_write_regs(dev, dev_addr, reg_addr, data, len, 0);
}

int8_t bmm150_aux_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    (void)intf_ptr; // unused
    
    i2c_acquire(dev);
    
    int8_t rslt = bmi160_aux_read(reg_addr, reg_data, length, &bmi);
    
    i2c_release(dev);
    
    return rslt;
}

int8_t bmm150_aux_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr){
    (void)intf_ptr; // unused
    
    i2c_acquire(dev);
    
    uint8_t* REG_data = (uint8_t*)reg_data;
    
    int8_t rslt = bmi160_aux_write(reg_addr, REG_data, length, &bmi);
    
    i2c_release(dev);
    
    return rslt;
}

void user_delay(uint32_t period)
{
    ztimer_sleep(ZTIMER_MSEC, period);
}

void user_delay_us(uint32_t period, void *intf_ptr)
{
    (void) intf_ptr;
    
    ztimer_sleep(ZTIMER_USEC, period);
}

// #define GATT_DEVICE_INFO_UUID                   0x180A
// #define GATT_MANUFACTURER_NAME_UUID             0x2A29
// #define GATT_MODEL_NUMBER_UUID                  0x2A24

#define STR_ANSWER_BUFFER_SIZE 4096

uint32_t t1, t2;
bmx280_t devbme;

int main(void)
{

    (void)puts("Welcome to RIOT!");

    puts("+------------Initializing------------+");
    
    init_bmi_bmm_Sensors();

    while (1)
    {
        //write_index = 0;
        /* rslt = bmi160_set_fifo_flush(&bmi);
        if (rslt != BMI160_OK)
        {
            printf("Error flushing BMI160 FIFO - %d\n \r", rslt);
            return 0;
        } */
        //t1 = xtimer_now_usec();
        //acquire_ACC_Values();
        //direct_read();
        //t2 = xtimer_now_usec();
        //printf("time: %d \n", (int)(t2 - t1) / 1000);
        acquire_read();
    }
    return 0;
}

void init_bmi_bmm_Sensors(void)
{
    i2c_init(dev);
    i2c_acquire(dev);

    /* Initialize your host interface to the BMI160 */

    /* This example uses I2C as the host interface */
    bmi.id = BMI160_I2C_ADDR;
    bmi.read = user_i2c_read;
    bmi.write = user_i2c_write;
    bmi.delay_ms = user_delay;
    bmi.intf = BMI160_I2C_INTF;
    
    /* The BMM150 API tunnels through the auxiliary interface of the BMI160 */
    /* Check the pins of the BMM150 for the right I2C address */
    // bmm.dev_id = BMM150_DEFAULT_I2C_ADDRESS;
    bmm.intf = BMM150_I2C_INTF;
    bmm.read = bmm150_aux_read;
    bmm.write = bmm150_aux_write;
    bmm.delay_us = user_delay_us;
    bmm.intf_ptr = &bmi;

    rslt = bmi160_init(&bmi);
    if (rslt == BMI160_OK)
    {
        printf("Success initializing BMI160 - Chip ID 0x%X\n \r", bmi.chip_id);
    }
    else if (rslt == BMI160_E_DEV_NOT_FOUND)
    {
        printf("Error initializing BMI160: device not found\n \r");
        return;
    }
    else
    {
        printf("Error initializing BMI160 - %d\n \r", rslt);
        return;
    }
    
    /* Configure the BMI160's auxiliary interface for the BMM150 */
    bmi.aux_cfg.aux_sensor_enable = BMI160_ENABLE;
    bmi.aux_cfg.aux_i2c_addr = BMI160_AUX_BMM150_I2C_ADDR; //bmm.dev_id;
    bmi.aux_cfg.manual_enable = BMI160_ENABLE; /* Manual mode */
    bmi.aux_cfg.aux_rd_burst_len = BMI160_AUX_READ_LEN_3; /* 8 bytes */
    
    rslt = bmi160_aux_init(&bmi);
    if (rslt == BMI160_OK)
    {
        printf("Success initializing BMi160 auxiliary interface for the BMM150");
    }
    else
    {
        printf("Error initializing BMi160 auxiliary interface for the BMM150 - %d\n \r", rslt);
        return;
    }
    
    rslt = bmm150_init(&bmm);
    if (rslt == BMI160_OK)
    {
        printf("Success initializing BMM150");
    }
    else
    {
        printf("Error initializing BMM150 - %d\n \r", rslt);
        return;
    }

    /* Select the power mode of accelerometer sensor */
    bmi.accel_cfg.power = BMI160_ACCEL_SUSPEND_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;
    
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error configuring BMI160 - %d\n \r", rslt);
        return;
    }
    
    
    /* Configure the magnetometer. The low power preset supports >300Hz in Forced mode */
    bmm_settings.preset_mode= BMM150_PRESETMODE_LOWPOWER;
    rslt = bmm150_set_presetmode(&bmm_settings, &bmm);
    if (rslt != BMM150_OK)
    {
        printf("Error configuring BMM150 preset mode - %d\n \r", rslt);
        return;
    }

    /* It is important that the last write to the BMM150 sets the forced mode.
     * This is because the BMI160 writes the last value to the auxiliary sensor 
     * after every read */
    bmm_settings.pwr_mode = BMM150_POWERMODE_FORCED;
    rslt = bmm150_set_op_mode(&bmm_settings, &bmm);
    if (rslt != BMM150_OK)
    {
        printf("Error configuring BMM150 power mode - %d\n \r", rslt);
        return;
    }
    
    uint8_t bmm150_data_start = BMM150_REG_DATA_X_LSB;
    bmi.aux_cfg.aux_odr = BMI160_AUX_ODR_200HZ;
    rslt = bmi160_set_aux_auto_mode(&bmm150_data_start, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error configuring BMI160 aux auto mode - %d\n \r", rslt);
        return;
    }
    

    /* Link the FIFO memory location */
    fifo_frame.data = fifo_buff;
    fifo_frame.length = FIFO_SIZE;
    bmi.fifo = &fifo_frame;
    
    /* Clear all existing FIFO configurations */
    rslt = bmi160_set_fifo_config(BMI160_FIFO_CONFIG_1_MASK, BMI160_DISABLE, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error clearing fifo - %d\n \r", rslt);
        return;
    }

    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_AUX;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error enabling fifo - %d\n \r", rslt);
        return;
    }
    
    i2c_release(dev);
}
/*
void acquire_ACC_Values(void)
{
    while(write_index < MAX_READINGS){*/
        /* It is VERY important to reload the length of the FIFO memory as after the
         * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
         * number of bytes read from the FIFO */
        /*bmi.fifo->length = FIFO_SIZE;
        i2c_acquire(dev);
        rslt = bmi160_get_fifo_data(&bmi);
        if (rslt != BMI160_OK)
        {
            printf("Error getting fifo data - %d\n \r", rslt);
            return;
        }
        i2c_release(dev);

        uint8_t acc_inst = ACC_FRAMES;

        rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
        if (rslt != BMI160_OK)
        {
            printf("Error extracting accel data - %d\n \r", rslt);
            return;
        }
        
        for (int j = 0; j < acc_inst && write_index < MAX_READINGS; j++)
        {
            readings_buffer[write_index].X_axis = accel_data[j].x;
            readings_buffer[write_index].Y_axis = accel_data[j].y;
            readings_buffer[write_index].Z_axis = accel_data[j].z;
            write_index++; // incrementa-se o indice do buffer a cada medida
        
        }
    }
}

void direct_read(void)
{
    for (int i = 0; i < MAX_READINGS; i++)
    {
        printf("%2.6f %2.6f %2.6f \n", ((float)readings_buffer[write_index].X_axis) / AC
                                     , ((float)readings_buffer[write_index].Y_axis) / AC
                                     , ((float)readings_buffer[write_index].Z_axis) / AC);
    }
}
*/
void acquire_read(void)
{
    /* It is VERY important to reload the length of the FIFO memory as after the
     * call to bmi160_get_fifo_data(), the bmi.fifo->length contains the
     * number of bytes read from the FIFO */
    bmi.fifo->length = FIFO_SIZE;
    i2c_acquire(dev);
    rslt = bmi160_get_fifo_data(&bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error getting fifo data - %d\n \r", rslt);
        return;
    }
    i2c_release(dev);

    uint8_t aux_inst = MAG_FRAMES;

    rslt = bmi160_extract_aux(aux_data, &aux_inst, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error extracting accel data - %d\n \r", rslt);
        return;
    }
        
    for (uint8_t i = 0; i < aux_inst; i++) {
        rslt = bmm150_aux_mag_data(&aux_data[0].data[0], &mag_data[i], &bmm);
        if (rslt != BMI160_OK)
        {
            printf("Error compensating magnetometer data - %d\n \r", rslt);
            return;
        }
            
        //mag_data[i] = bmm.data;
            
        printf("%2.6f %2.6f %2.6f \n", mag_data[i].x, mag_data[i].y, mag_data[i].z);
    }
        
}
