/*
 * Copyright (C) 2017 Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_nz32-sc151
 * @{
 *
 * @file
 * @brief       sx127x configuration for nz32-sc15 using modtronix inAir9
 *              LoRa module
 * @author      Francisco Molina <francisco.molina@inria.cl>
*/

#ifndef SX127X_PARAMS_H
#define SX127X_PARAMS_H

#include "sx127x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Board specific SX127X configuration
 */
static const sx127x_params_t sx127x_params[] =
{
    {
        .spi       = SPI_DEV(0),
        .nss_pin   = GPIO_PIN(0,  26),
        .reset_pin = GPIO_PIN(1,  15),
        .dio0_pin  = GPIO_PIN(0,  12),
        .dio1_pin  = GPIO_PIN(0,  14), // not routed in the board
        .dio2_pin  = GPIO_PIN(1,  8), // not routed in the board
        .dio3_pin  = GPIO_PIN(1, 9), // not routed in the board
        .paselect = SX127X_PARAM_PASELECT,
#if defined(SX127X_USE_TX_SWITCH) || defined(SX127X_USE_RX_SWITCH)
        /* use a gpio to control the antenna tx switch
         * (the value is inverted in the board, so we
         *  call the **receive pin** to achieve the intended functionality) */
        .rx_switch_pin  = GPIO_PIN(0, 2)
#endif
    }
};

#ifdef __cplusplus
}
#endif

#endif /* SX127X_PARAMS_H */
/** @} */
