/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
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
/** @file
 *
 * @defgroup bootloader_secure_ble main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file for secure DFU.
 *
 */

#include <stdint.h>
#include "boards.h"
#include "nrf_mbr.h"
#include "nrf_bootloader.h"
#include "nrf_bootloader_app_start.h"
#include "nrf_bootloader_dfu_timers.h"
#include "nrf_dfu.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "app_error.h"
#include "app_error_weak.h"
#include "nrf_bootloader_info.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "drv_sx1509.h"

#ifdef BOARD_PCA20020

/**< TWI instance used to control the SX1509. */
static const nrf_drv_twi_t m_twi_instance = NRF_DRV_TWI_INSTANCE(0);

/**< SX1509 TWI configuration. */
static drv_sx1509_conf_t m_sx1509;

#endif

static void on_error(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(100);
#endif
#ifdef NRF_DFU_DEBUG_VERSION
    NRF_BREAKPOINT_COND;
#endif
    NVIC_SystemReset();
}


void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NRF_LOG_ERROR("%s:%d", p_file_name, line_num);
    on_error();
}


void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    NRF_LOG_ERROR("Received a fault! id: 0x%08x, pc: 0x%08x, info: 0x%08x", id, pc, info);
    on_error();
}


void app_error_handler_bare(uint32_t error_code)
{
    NRF_LOG_ERROR("Received an error: 0x%08x!", error_code);
    on_error();
}

/**
 * @brief Function notifies certain events in DFU process.
 */
static void dfu_observer(nrf_dfu_evt_type_t evt_type)
{
    switch (evt_type)
    {
        case NRF_DFU_EVT_DFU_FAILED:
        case NRF_DFU_EVT_DFU_ABORTED:
        case NRF_DFU_EVT_DFU_INITIALIZED:
        case NRF_DFU_EVT_TRANSPORT_DEACTIVATED:
#if LEDS_NUMBER > 0
            bsp_board_init(BSP_INIT_LEDS);
            bsp_board_led_on(BSP_BOARD_LED_0);
            bsp_board_led_on(BSP_BOARD_LED_1);
            bsp_board_led_off(BSP_BOARD_LED_2);
#endif

#ifdef BOARD_PCA20020
            // Enable only red LEDs with configured animation.
            drv_sx1509_reg_set(&m_sx1509, 0x11, 0b01111111);
#endif
            break;

        case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
#if LEDS_NUMBER > 0
            bsp_board_led_off(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
#endif

#ifdef BOARD_PCA20020
            // Enable only green LEDs with configured animation.
            drv_sx1509_reg_set(&m_sx1509, 0x11, 0b11011111);
#endif
            break;

        default: break;
    }
}

#ifdef BOARD_PCA20020
/**
 * @brief Function initializes the led driver used to indicate DFU status.
 */
static void board_init(void)
{
    ret_code_t err_code;

    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = APP_IRQ_PRIORITY_LOW
    };

    m_sx1509.p_twi_instance = &m_twi_instance;
    m_sx1509.p_twi_config   = &twi_config;
    m_sx1509.address        = 0x3E;

    // Enable SX1509 power supply.
    nrf_gpio_cfg_output(VDD_PWR_CTRL);
    nrf_gpio_pin_set(VDD_PWR_CTRL);
    nrf_delay_ms(5);

    // Initialize TWI communication.
    err_code = drv_sx1509_open(&m_sx1509);
    APP_ERROR_CHECK(err_code);

    // Disable input buffer.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_InputDisableA, 0b11100000);
    // Disable pull-up.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_PullUpA, 0b00000000);
    // Enable open drain.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_OpenDrainA, 0b11100000);
    // Set direction to output.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_DirA, 0b00011111);
    // Enable oscillator.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_Clock, 0b01000000);
    // Configure LED driver clock and mode.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_Misc, 0b10111000);
    // Enable LED driver operation.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_LEDDriverEnableA, 0b11100000);

    // Configure LED driver parameters for green channel.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TOn5  , 0b00000011);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_IOn5  , 0b11111111);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_Off5  , 0b00111000);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TRise5, 0b00000001);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TFall5, 0b00000001);

    // Configure LED driver parameters for blue channel.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TOn6  , 0b00000011);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_IOn6  , 0b11111111);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_Off6  , 0b00111000);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TRise6, 0b00000001);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TFall6, 0b00000001);

    // Configure LED driver parameters for red channel.
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TOn7  , 0b00000011);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_IOn7  , 0b11111111);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_Off7  , 0b00111000);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TRise7, 0b00000001);
    drv_sx1509_reg_set(&m_sx1509, SX1509_REG_TFall7, 0b00000001);
}
#endif


/**@brief Function for application main entry. */
int main(void)
{
    uint32_t ret_val;

    // Must happen before flash protection is applied, since it edits a protected page.
    nrf_bootloader_mbr_addrs_populate();

    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE);
    APP_ERROR_CHECK(ret_val);

    (void) NRF_LOG_INIT(nrf_bootloader_dfu_timer_counter_get);
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("Inside main");

#ifdef BOARD_PCA20020
    board_init();
#endif

    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);

    NRF_LOG_FLUSH();

    NRF_LOG_ERROR("After main, should never be reached.");
    NRF_LOG_FLUSH();

    APP_ERROR_CHECK_BOOL(false);
}

/**
 * @}
 */
