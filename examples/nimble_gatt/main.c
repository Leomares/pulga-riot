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
#include "bmx280_params.h"
#include "bmx280.h"

#include <stdint.h>
#include "event/timeout.h"
// #include "net/bluetil/ad.h"
#include "timex.h"
/* Macros for frames to be read */

#define ACC_FRAMES 10 /* 10 Frames are available every 100ms @ 100Hz */ // Datasheet says you can take measurementes at 1600 Hz
/* 10 frames containing a 1 byte header, 6 bytes of accelerometer,
 * 6 bytes of gyroscope and 8 bytes of magnetometer data. This results in
 * 21 bytes per frame. Additional 40 bytes in case sensor time readout is enabled */
#define FIFO_SIZE 250

#define TAMANHO_MEDIDA (3 * sizeof(int16_t) + sizeof(int))
#define CONTADOR 15 // QUANTIDADE DE MEDIDAS DO DUMP

/* Variable declarations */
struct bmi160_dev bmi;

uint8_t fifo_buff[FIFO_SIZE];
struct bmi160_fifo_frame fifo_frame;
struct bmi160_sensor_data accel_data[ACC_FRAMES];

int8_t rslt;

// typedef struct
// {
//     int16_t accelerometerX;
//     int16_t accelerometerY;
//     int16_t accelerometerZ;
//     uint16_t dummy;
// } reading_t;

typedef struct
{
    int16_t X_axis;
    int16_t Y_axis;
    int16_t Z_axis;
    int timestamp;
} leitura;

#define MAX_READINGS 11000

leitura readings_buffer[MAX_READINGS];
size_t rlen = 0;

int dump_index = 0;

void init_bmiSensor(void);
void direct_read(void);
void do_read(void);
void log_readings(void);
void acquire_ACC_Values(void);

uint16_t latest_measurement = 0; // Ring buffer index
uint16_t get_ring_index(void);
void add_ring_index(void);

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

void user_delay(uint32_t period)
{
    ztimer_sleep(ZTIMER_MSEC, period);
}

/* accel params and conversion constants */
// #define AC 2048.0 // for 16G
#define AC 8192.0 // for 16G
// #define AC 16384.0 // for 2G
/* gyro params and conversion constants */
#define GC 16.4 // for 2000 DPS
// #define GC 131.2 // for 250 DPS

// #define GATT_DEVICE_INFO_UUID                   0x180A
// #define GATT_MANUFACTURER_NAME_UUID             0x2A29
// #define GATT_MODEL_NUMBER_UUID                  0x2A24

#define STR_ANSWER_BUFFER_SIZE 4096

uint32_t t1, t2;
bmx280_t devbme;

int main(void)
{

    (void)puts("Welcome to RIOT!");

    init_bmiSensor();

    puts("+------------Initializing------------+");
    switch (bmx280_init(&devbme, &bmx280_params[0]))
    {
    case BMX280_ERR_BUS:
        puts("[Error] Something went wrong when using the I2C bus");
        return 1;
    case BMX280_ERR_NODEV:
        puts("[Error] Unable to communicate with any BMX280 device");
        return 1;
    default:
        /* all good -> do nothing */
        break;
    }

    while (1)
    {
        t1 = xtimer_now_usec();
        acquire_ACC_Values();
        direct_read();
        t2 = xtimer_now_usec();
        printf("time: %d \n", (int)(t2 - t1) / 1000);
        // do_read(); // aqui iremos adicionar os dados no final do ring buffer e incrementalo
        // log_readings();
    }
    return 0;
}

void direct_read(void)
{
    for (int i = 0; i < ACC_FRAMES; i++)
    {
        printf("%2.6f ", ((float)accel_data[i].x) / AC);
        printf("%2.6f ", ((float)accel_data[i].y) / AC);
        printf("%2.6f ", ((float)accel_data[i].z) / AC);
        printf("%2.2f ", bmx280_read_temperature(&devbme) / 100.0);
        printf("%d ", (int)(xtimer_now_usec() / 1000));
        printf("\n");
    }
}

void do_read(void)
{
    uint16_t aux = get_ring_index(); // supoes-se que o index nunca vai ser um valor proibido nunca <0 e nem >= MAX READINGS
    uint16_t i, j;
    for (i = aux, j = 0; i < (aux + ACC_FRAMES) && j < ACC_FRAMES; i++, j++)
    {
        readings_buffer[i].X_axis = accel_data[j].x;
        readings_buffer[i].Y_axis = accel_data[j].y;
        readings_buffer[i].Z_axis = accel_data[j].z;
        readings_buffer[i].timestamp = (int)(xtimer_now_usec() / 1000);
        add_ring_index(); // incrementa-se o indice do ring buffer a cada medida
    }
}

void log_readings(void)
{
    int i = get_ring_index() - 1;
    // printf("[Acc_readings] readings_buffer[%d]: ", i);
    // printf("Acc_x: %2.6f ", ((float)readings_buffer[i].X_axis) / AC);
    printf("%2.6f ", ((float)readings_buffer[i].X_axis) / AC);
    // printf("Acc_y: %2.6f ", ((float)readings_buffer[i].Y_axis) / AC);
    printf("%2.6f ", ((float)readings_buffer[i].Y_axis) / AC);
    // printf("Acc_z: %2.6f ", ((float)readings_buffer[i].Z_axis) / AC);
    printf("%2.6f ", ((float)readings_buffer[i].Z_axis) / AC);
    // printf("Acc_timeStamp: %d ", (readings_buffer[i].timestamp));
    printf("%d ", (readings_buffer[i].timestamp));
    // printf("\n \r");
    printf("\n");
}

void init_bmiSensor(void)
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

    /* Select the Output data rate, range of accelerometer sensor */
    bmi.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    bmi.accel_cfg.range = BMI160_ACCEL_RANGE_4G;
    // bmi.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    bmi.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    bmi.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    // bmi.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    // bmi.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    //  bmi.gyro_cfg.range = BMI160_GYRO_RANGE_250_DPS;
    // bmi.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    bmi.gyro_cfg.power = BMI160_GYRO_SUSPEND_MODE;

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error configuring BMI160 - %d\n \r", rslt);
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

    uint8_t fifo_config = BMI160_FIFO_HEADER | BMI160_FIFO_ACCEL | BMI160_FIFO_GYRO;
    rslt = bmi160_set_fifo_config(fifo_config, BMI160_ENABLE, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error enabling fifo - %d\n \r", rslt);
        return;
    }
    /* Check rslt for any error codes */
    i2c_release(dev);
}

void acquire_ACC_Values(void)
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

    uint8_t acc_inst = ACC_FRAMES;

    rslt = bmi160_extract_accel(accel_data, &acc_inst, &bmi);
    if (rslt != BMI160_OK)
    {
        printf("Error extracting accel data - %d\n \r", rslt);
        return;
    }
}

uint16_t get_ring_index(void)
{
    return latest_measurement;
}

void add_ring_index(void)
{
    // latest_measurement = rlen < MAX_READINGS ? rlen++ : 0;
    if (rlen < MAX_READINGS - 1)
    {
        rlen++;
        latest_measurement = rlen;
    }
    else if (rlen == MAX_READINGS - 1)
    {
        latest_measurement = 0;
        rlen++;
    }
    else if (latest_measurement == MAX_READINGS - 1)
    {
        latest_measurement = 0;
    }
    else
    {
        latest_measurement++;
    }
}