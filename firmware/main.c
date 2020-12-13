/*
 * TSDZ2 EBike wireless bootloader
 *
 * Copyright (C) rananna, 2020
 *
 * Released under the GPL License, Version 3
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
#include "nrf_drv_rtc.h"

#define SCHED_MAX_EVENT_DATA_SIZE MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, 0)
#define SCHED_QUEUE_SIZE 20

#define BUTTON_DETECTION_DELAY APP_TIMER_TICKS(50)
#define BUTTON_DFU_WAIT APP_TIMER_TICKS(10000) //ms to wait for dfu mode to initiate
APP_TIMER_DEF(m_timer_button_long_press_timeout);

const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);

bool g_start_bootloader = false;

static void lfclk_start(void)
{
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth;
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
static bool gpio_init(void)
{
  ret_code_t err_code;
  bool bootloader_pin_pressed = false;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  // enable button release interrupt only if the pin is pressed
  if (nrf_gpio_pin_read(BUTTON_1) == 0)
  {
    err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, button_released);
    nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
    APP_ERROR_CHECK(err_code);

    bootloader_pin_pressed = true;
  }

  // enable button release interrupt only if the pin is pressed
  if (nrf_gpio_pin_read(PLUS__PIN) == 0)
  {
    err_code = nrf_drv_gpiote_in_init(PLUS__PIN, &in_config, button_released);
    nrf_drv_gpiote_in_event_enable(PLUS__PIN, true);
    APP_ERROR_CHECK(err_code);

    bootloader_pin_pressed = true;
  }

  return bootloader_pin_pressed;
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
  if (int_type == NRF_DRV_RTC_INT_COMPARE0)
  {
    g_start_bootloader = true;
  }
}

static void lfclk_config(void)
{
  ret_code_t err_code = nrf_drv_clock_init();
  APP_ERROR_CHECK(err_code);

  nrf_drv_clock_lfclk_request(NULL);
}

static void rtc_config(void)
{
  uint32_t err_code;

  //Initialize RTC instance
  nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
  config.prescaler = 4095;
  config.interrupt_priority = 7;
  err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
  APP_ERROR_CHECK(err_code);

  //Set compare channel to trigger interrupt after COMPARE_COUNTERTIME seconds
  err_code = nrf_drv_rtc_cc_set(&rtc, 0, 10 * 8,true);
  APP_ERROR_CHECK(err_code);

  //Power on RTC instance
  nrf_drv_rtc_enable(&rtc);
}

static void rtc_uninit(void)
{
  nrf_drv_rtc_uninit(&rtc);
}

/**@brief Function for application main entry. */
int main(void)
{
  ret_code_t ret_val;
  bool bootloader_pin_pressed = false;

  leds_init(); //turn on the red led

  // check if bootloader start flag was set before previous reset
  if (nrf_power_gpregret_get() == BOOTLOADER_DFU_START)
  {
    g_start_bootloader = true;
  }
  else
  // check for timeout or bootloader pins
  {
    // init GPIOS and init the release interrupts if applicable
    bootloader_pin_pressed = gpio_init();

    // if at least one bootloader button is pressed 
    if (bootloader_pin_pressed)
    {
      lfclk_start(); // start the low freq clock

      // start RTC timer timeout
      rtc_config();

      // will enter in low power mode and block
      // Will unblock and move forward only when timeout happens or one of the bootloader buttons are released
      __SEV();
      __WFE();
      __WFE();
    }
  }

  rtc_uninit();

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
