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
#include "nrf_sdh.h"
//#include "peripheral.h" // header for the functions here
#include "boards.h"
#include "app_button.h"
#include "app_scheduler.h"
//#include "nrf_pwr_mgmt.h"
#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, 0)
#define SCHED_QUEUE_SIZE 20

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)
#define BUTTON_DFU_WAIT APP_TIMER_TICKS(10000) //ms to wait for dfu mode to initiate
APP_TIMER_DEF(m_timer_button_long_press_timeout);

static void lfclk_start(void)
{
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal;
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
    }
}
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

static void timer_button_long_press_timeout_handler(void *p_context)
{
    ret_code_t ret_val;
    UNUSED_PARAMETER(p_context);
    //timed out so reboot into dfu mode
    ret_val = app_timer_stop(m_timer_button_long_press_timeout); //stop the long press timerf
    APP_ERROR_CHECK(ret_val);
    bsp_board_led_off(BSP_BOARD_LED_1);
    bsp_board_led_on(BSP_BOARD_LED_0);
    nrf_power_gpregret_set(BOOTLOADER_DFU_START); //set the dfu register
    nrf_delay_ms(1000);                           //wait for write to complete
    do_reset();
}
static void leds_init(void)
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
}
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_create(&m_timer_button_long_press_timeout,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                timer_button_long_press_timeout_handler);
    APP_ERROR_CHECK(err_code);
    //start the button timer
    err_code = app_timer_start(m_timer_button_long_press_timeout, BUTTON_DFU_WAIT, NULL); //start the long press timerf
    APP_ERROR_CHECK(err_code);
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
    ret_code_t err_code;

    switch (action)
    {
    case NRF_GPIOTE_POLARITY_LOTOHI:
    {
        bsp_board_led_off(BSP_BOARD_LED_1);
        bsp_board_led_off(BSP_BOARD_LED_0);
        //turn off the button timer
        err_code = app_timer_stop(m_timer_button_long_press_timeout); //stop the long press timerf
        APP_ERROR_CHECK(err_code);
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
    leds_init(); //turn on the red led
    // if bootloader is about to enter dfu mode,
    //don't check for button press and immediately go into DFU
    if (nrf_power_gpregret_get() != BOOTLOADER_DFU_START)
    {

        gpio_init(); //set the gpio interrupt
        //check if bootloader button pressed
        //Casainho - note that I added GPIO pin 9 as an additional check for TSDZ2 wireless. 
        //the remote already uses the PLUS__KEY to enter DFU
        // you could add an additional switch to pin 9 if the board pin is unreliable
       // ie: if ((nrf_gpio_pin_read(9) == 0) || (nrf_gpio_pin_read(BUTTON_1) == 0)) // button pressed
       if (nrf_gpio_pin_read(BUTTON_1) == 0) // button pressed
        {
            lfclk_start(); //start the low freq clock
            timers_init(); //start the button timer
                           //scheduler needed for app_timer and app_button event processing with bootloader
            APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
            while (true) //loop while the button is pressed
            {
                app_sched_execute(); //execute the timer events
                                     //  __WFE();             //low power mode
                // Clear the internal event register.
                //  __SEV();
                //  __WFE();
            }
            // if the button is released before timeout, button_released() is called and the board resets into DFU
        }
    }
    // else go into the bootloader
    // Protect MBR and bootloader code from being overwritten.
    ret_val = nrf_bootloader_flash_protect(0, MBR_SIZE, false);
    APP_ERROR_CHECK(ret_val);
    ret_val = nrf_bootloader_flash_protect(BOOTLOADER_START_ADDR, BOOTLOADER_SIZE, false);
    APP_ERROR_CHECK(ret_val);

    ret_val = app_timer_init();
    APP_ERROR_CHECK(ret_val);

    ret_val = nrf_bootloader_init(dfu_observer);
    APP_ERROR_CHECK(ret_val);
    //if the program is here there was either
    //-no DFU requested in the bootloader
    // or the DFU module detected no ongoing DFU operation and found a valid main application.
    //so, load the installed application
    bsp_board_led_off(BSP_BOARD_LED_1);
    bsp_board_led_off(BSP_BOARD_LED_0);
    nrf_bootloader_app_start();
}
