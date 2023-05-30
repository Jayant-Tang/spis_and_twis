/**
 * Copyright (c) 2015 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_config.h"
#include "nrf_drv_spis.h"
#include "nrf_drv_twis.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


// SPIS
#define SPIS_INSTANCE 2 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */


static uint8_t       m_tx_buf[20] = {0};
static uint8_t       m_rx_buf[20] = {0};
static const uint8_t m_length = sizeof(m_tx_buf);
static uint8_t m_rx_length = {0};

static volatile bool spis_xfer_done = false; /**< Flag used to indicate that SPIS instance completed the transfer. */

// TWIS
#define TWIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_twis_t m_twis = NRF_DRV_TWIS_INSTANCE(TWIS_INSTANCE);
static uint8_t m_twis_buf[20] = {0};

/**
 * @brief SPIS user event handler.
 *
 * @param event
 */
void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        spis_xfer_done = true;
        m_rx_length = event.rx_amount;
        NRF_LOG_INFO("SPI_DATA_RECEIVED:");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, event.rx_amount);
    }
}

enum custom_cmd_t {
    // FLASH_WRITE = 0x01,
    // FLASH_READ = 0x02,
    SPIM_WRITE = 0x03,
    SPIM_READ = 0x04,
    TWI_WRITE = 0x05,
    TWI_READ = 0x06,
};

void spis_task()
{
    static uint8_t spis_storage[20] = {0};

    if (!spis_xfer_done){
        APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
        return;
    }

    uint8_t cmd = m_rx_buf[0];
    uint8_t len = m_rx_buf[1];
    uint8_t *data = &m_rx_buf[2];

    switch(cmd){
        case SPIM_WRITE:
            if (len != m_rx_length - 2) {
                NRF_LOG_WARNING("write len is not match! paylod_len=%d, len=%d", m_rx_length - 2, len);
            }
            memcpy(spis_storage, data, len);
            break;
        case SPIM_READ:
            NRF_LOG_INFO("SPI_READ:");
            NRF_LOG_HEXDUMP_INFO(spis_storage, len);
            memcpy(m_tx_buf, spis_storage, len);
            break;
        default:
            break;
    }
    memset(m_rx_buf, 0, m_length);
    spis_xfer_done= false;
    APP_ERROR_CHECK(nrf_drv_spis_buffers_set(&spis, m_tx_buf, m_length, m_rx_buf, m_length));
}

static void twis_event_handler(nrf_drv_twis_evt_t const * const p_event)
{
    switch (p_event->type)
    {
    case TWIS_EVT_READ_REQ:
        if(p_event->data.buf_req){
            nrf_drv_twis_tx_prepare(&m_twis, m_twis_buf, sizeof(m_twis_buf));
        }
        break;
    case TWIS_EVT_READ_DONE:
        NRF_LOG_INFO("TWIS_EVT_READ_DONE");
        break;
    case TWIS_EVT_WRITE_REQ:
        if(p_event->data.buf_req){
            nrf_drv_twis_rx_prepare(&m_twis, m_twis_buf, sizeof(m_twis_buf));
        }
        break;
    case TWIS_EVT_WRITE_DONE:
        NRF_LOG_INFO("TWIS_EVT_WRITE_DONE")
        break;

    case TWIS_EVT_READ_ERROR:
    case TWIS_EVT_WRITE_ERROR:
    case TWIS_EVT_GENERAL_ERROR:
        // m_error_flag = true;
        break;
    default:
        break;
    }
}

int main(void)
{
    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    NRF_POWER->TASKS_CONSTLAT = 1;

    //bsp_board_init(BSP_INIT_LEDS);


    // init spis
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("SPIS and TWIS example");
    NRF_LOG_FLUSH();

    nrf_drv_spis_config_t spis_config = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spis_config.csn_pin               = 3;
    spis_config.sck_pin               = 4;
    spis_config.mosi_pin              = 28;
    spis_config.miso_pin              = 29;
    
    APP_ERROR_CHECK(nrf_drv_spis_init(&spis, &spis_config, spis_event_handler));

    // init twis
    const nrf_drv_twis_config_t config =
    {
        .addr               = {0xbe, 0xef},
        .scl                = 33, // P1.01
        .scl_pull           = NRF_GPIO_PIN_PULLUP,
        .sda                = 34, // P1.02
        .sda_pull           = NRF_GPIO_PIN_PULLUP,
        .interrupt_priority = APP_IRQ_PRIORITY_HIGH
    };

    do
    {
        int ret = nrf_drv_twis_init(&m_twis, &config, twis_event_handler);
        if (NRF_SUCCESS != ret)
        {
            break;
        }

        nrf_drv_twis_enable(&m_twis);
    }while (0);


    while (1)
    {

        spis_task();
        NRF_LOG_FLUSH();

        while (!spis_xfer_done)
        {
            __WFE();
        }

         //bsp_board_led_invert(BSP_BOARD_LED_0);
    }
}
