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

#include <stdint.h>
#include "event/timeout.h"
#include "timex.h"

#include <math.h>
#include "lstm_weights.h"

#define LSTM_units 32

float sigmoid(float x){
    return 1 / (1 + exp(-x));
}

void Pulga_LSTM(float X[], int input_size, float output[]){
    float h_t[LSTM_units] = {0};
    float c_t[LSTM_units] = {0};

    for(int t = 0; t < input_size; t++){
        float f_t[LSTM_units] = {0};
        float i_t[LSTM_units] = {0};
        float o_t[LSTM_units] = {0};
        float c_hat_t[LSTM_units] = {0};

        float x_t = X[t];

        for(int i = 0; i < LSTM_units; i++){
            for(int j = 0; j < LSTM_units; j++){
                f_t[i] += h_t[j] * U_f[j][i];
            }
            f_t[i] += x_t * W_f[i];
            f_t[i] += b_f[i];
            f_t[i] = sigmoid(f_t[i]);
        }

        for(int i = 0; i < LSTM_units; i++){
            for(int j = 0; j < LSTM_units; j++){
                i_t[i] += h_t[j] * U_i[j][i];
            }
            i_t[i] += x_t * W_i[i];
            i_t[i] += b_i[i];
            i_t[i] = sigmoid(i_t[i]);
        }

        for(int i = 0; i < LSTM_units; i++){
            for(int j = 0; j < LSTM_units; j++){
                o_t[i] += h_t[j] * U_o[j][i];
            }
            o_t[i] += x_t * W_o[i];
            o_t[i] += b_o[i];
            o_t[i] = sigmoid(o_t[i]);
        }

        for(int i = 0; i < LSTM_units; i++){
            for(int j = 0; j < LSTM_units; j++){
                c_hat_t[i] += h_t[j] * U_c[j][i];
            }
            c_hat_t[i] += x_t * W_c[i];
            c_hat_t[i] += b_c[i];
            c_hat_t[i] = tanh(c_hat_t[i]);
        }

        for(int i = 0; i < LSTM_units; i++){
            c_t[i] = f_t[i] * c_t[i] + i_t[i] * c_hat_t[i];
        }

        for(int i = 0; i < LSTM_units; i++){
            h_t[i] = o_t[i] * tanh(c_t[i]);
        }
    }
    for(int i = 0; i < LSTM_units; i++){
        output[i] = h_t[i];
    }
}

int main(void)
{

    (void)puts("Welcome to RIOT!");
    
    float X[5] = {1.0,1.0,1.0,1.0,1.0};
    float output[LSTM_units];

    Pulga_LSTM(X, 5, output);

    for(int i = 0; i < LSTM_units; i++){
        printf("\n \r %f", output[i]);
    }
    
    printf("\n \r \n \r \n \r");
    
    while (1)
    {
    }
    return 0;
}
