/*
 * TSDZ2 EBike wireless bootloader
 *
 * Copyright (C) rananna, 2020
 *
 * Released under the GPL License, Version 3
 */
/*
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
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

#include <stdint.h>
#include "boards.h"
#include "bsp.h"
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
#include "nrfx.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrf_power.h"
#include "custom_board.h"
#include "pins.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#define BUTTON_DFU_WAIT 10000 //ms to wait for dfu mode to initiate

static void do_reset(void)
{
    NRF_LOG_FINAL_FLUSH();

#if NRF_MODULE_ENABLED(NRF_LOG_BACKEND_RTT)
    // To allow the buffer to be flushed by the host.
    nrf_delay_ms(500);
#endif

    nrf_delay_ms(NRF_BL_RESET_DELAY_MS);

    NVIC_SystemReset();
}

static void on_error(void)
{
    do_reset();
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t *p_file_name)
{

    on_error();
}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{

    on_error();
}

void app_error_handler_bare(uint32_t error_code)
{

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
        if (LEDS_NUMBER > 0)
        {
            bsp_board_led_on(BSP_BOARD_LED_0);
            if (LEDS_NUMBER <= 2)
            {
                bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            }
            else
            {
                bsp_board_led_on(BSP_BOARD_LED_1);
                bsp_board_led_off(BSP_BOARD_LED_2);
            }
        }
        break;
    case NRF_DFU_EVT_TRANSPORT_ACTIVATED:
        if (LEDS_NUMBER > 2)
        {
            bsp_board_led_off(BSP_BOARD_LED_0);
            bsp_board_led_off(BSP_BOARD_LED_1);
            bsp_board_led_on(BSP_BOARD_LED_2);
            bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
        }
        break;
    case NRF_DFU_EVT_DFU_STARTED:
        break;
    default:
        break;
    }
}
void button_released(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    switch (action)
    {
    case NRF_GPIOTE_POLARITY_LOTOHI:
    {
        bsp_board_led_off(BSP_BOARD_LED_1);
        do_reset(); //reset and start over
    }
    break;
    case NRF_GPIOTE_POLARITY_HITOLO:
        break;
    case NRF_GPIOTE_POLARITY_TOGGLE:
        break;
    }
}
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PLUS__PIN, &in_config, button_released);
    nrf_drv_gpiote_in_event_enable(PLUS__PIN, true);

    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, button_released);
    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry. */
int main(void)
{

    ret_code_t ret_val;

    if (LEDS_NUMBER > 0)
    {

        bsp_board_init(BSP_INIT_LEDS);
        ret_val = bsp_init(BSP_INIT_LEDS, NULL);
        APP_ERROR_CHECK(ret_val);
    }
    // turn on the led to indicate we are in the bootloader
    bsp_board_led_on(BSP_BOARD_LED_1); //indicate that the bootloader is active

    // if bootloader is about to enter dfu mode, don't check buttons
    if (nrf_power_gpregret_get() != BOOTLOADER_DFU_START)
    {
        gpio_init();
        if ((nrf_gpio_pin_read(PLUS__PIN) == 0) || (nrf_gpio_pin_read(BUTTON_1) == 0)) // button pressed
        {
            //start a timeout for DFU mode
            // if the button is released before timeout, button_released() is called and the board resets
            nrf_delay_ms(BUTTON_DFU_WAIT);                //wait 10 seconds before going to dfu mode
            nrf_power_gpregret_set(BOOTLOADER_DFU_START); //set the dfu register
            nrf_delay_ms(1000);                           //wait for write to complete
            do_reset();                                   //reset and go to dfu mode
        }
    }

// Protect MBR and bootloader code from being overwritten.
ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
APP_ERROR_CHECK(ret_val);
ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
APP_ERROR_CHECK(ret_val);

ret_val = app_timer_init();
APP_ERROR_CHECK(ret_val);

// Initiate the bootloader
ret_val = nrf_bootloader_init(dfu_observer);
APP_ERROR_CHECK(ret_val);
//if the program is here there was either
//-no DFU requested in the bootloader
// or the DFU module detected no ongoing DFU operation and found a valid main application.
//so, load the installed application
bsp_board_led_off(BSP_BOARD_LED_1);
nrf_bootloader_app_start();
}
