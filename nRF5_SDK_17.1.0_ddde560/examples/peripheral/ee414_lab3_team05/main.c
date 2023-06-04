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
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_twi_sensor.h"
#include <stdio.h>
#include "nrf_drv_gpiote.h"
#include "nrf_drv_timer.h"

#include "oled_display.h"
#include "APDS9960.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1
#endif
#define TIMER_INSTANCE_ID 0
/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127
#define PIN_VDD_ENV 22  // 0<<5||22 for P0.22
#define PIN_R_PULLUP 32 // 1<<5||0 for P1.0

/* TWI instance. */

static volatile bool m_xfer_done = false;

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(TIMER_INSTANCE_ID);
NRF_TWI_MNGR_DEF(m_twi_oled_mngr, 5, 1);
NRF_TWI_SENSOR_DEF(m_twi_oled, &m_twi_oled_mngr, 128);

bool enableGesture(void);

void LED_SLEEP_HANDLER(nrf_timer_event_t event_type, void* p_context)
{
    // Handler to put the LED into sleep state after 5 seconds
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            oled_command(SET_DISPLAY_OFF, &m_twi_oled);
            nrf_drv_timer_compare_int_disable(&m_timer, NRF_TIMER_CC_CHANNEL0);
            break;
        default:
            break;
    }
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // When Interrupt signal is sent by APDS-9960, 
    // the processor sends the command to enable gesture mode
    // and sense the gesture
}
/**
 * @brief Function for configuring: APDS_9960_INT_PIN pin for input,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);


    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(APDS_9960_INT_PIN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(APDS_9960_INT_PIN, true);
}

/**
 * @brief TWI initialization.
 */
 void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context)
{
    switch (p_event->type)
    {
    case NRF_DRV_TWI_EVT_DONE:
        if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
        {
        }

        m_xfer_done = true;
        break;
    default:
        break;
    }
}

void twi_init_sensor(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL1_PIN,
      .sda = ARDUINO_SDA1_PIN,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, twi_handler, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
  // To secure correct signal levels on the pins used by the TWI master when the system is in OFF mode, and
  // when the TWI master is disabled, SDA and SCL pins must be configured in the GPIO peripheral as follows:
  // SCL As specified in PSEL.SCL: Input H0D1
  // SDA As specified in PSEL.SDA: Input H0D1
  nrf_gpio_cfg(ARDUINO_SDA1_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(ARDUINO_SCL1_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
}

void twi_init_oled(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL2_PIN,
      .sda = ARDUINO_SDA2_PIN,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_twi_mngr_init(&m_twi_oled_mngr, &twi_config);
  APP_ERROR_CHECK(err_code);
  err_code = nrf_twi_sensor_init(&m_twi_oled);
  APP_ERROR_CHECK(err_code);

  // To secure correct signal levels on the pins used by the TWI master when the system is in OFF mode, and
  // when the TWI master is disabled, SDA and SCL pins must be configured in the GPIO peripheral as follows:
  // SCL As specified in PSEL.SCL: Input H0D1
  // SDA As specified in PSEL.SDA: Input H0D1
  nrf_gpio_cfg(ARDUINO_SDA2_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(ARDUINO_SCL2_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP,
      NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);
}

/*
 * dev_addr -> Device address
 * reg_adddr -> Register address
 * data -> Buffer which holds data to be written
 * length -> Length of data to be written
 */
int8_t APDS9960_WRITE(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  /* Write the data to APDS-9960 sensor by I2C protocol
   * reg_addr -> Register address
   * data -> Buffer where data read from TWI will be stored
   * length -> Length of data to be write
   */
    uint32_t err_code;
    uint8_t buffer[255] = {0};
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, length);
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, APDS_9960_ADDR, buffer, length + 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

   return err_code;
}


int8_t APDS9960_READ(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  /* Read the data in APDS-9960 sensor by I2C protocol
   * reg_addr -> Register address
   * data -> Buffer where data read from TWI will be stored
   * length -> Length of data to be read
   */
    uint32_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, APDS_9960_ADDR, &reg_addr, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, APDS_9960_ADDR, data, length);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    return err_code;

}

bool getGSTATUS(uint8_t *r)
{
      // Get the content from GSTATUS register
      uint8_t err_code = APDS9960_READ(GSTATUS, (uint8_t*) r, 1);
      if (err_code == 0) return true;
      return false;
}

bool getGFLVL(uint8_t *r)
{
  // Get the content from GFLVL register
  uint8_t err_code = APDS9960_READ(GFLVL, (uint8_t*) r, 1);
  if (err_code == 0) return true;
  return false;
}

bool getENABLE(uint8_t *r)
{
  // Get the content from ENABLE register
  uint8_t err_code = APDS9960_READ(ENABLE, (uint8_t*) r, 1);
  if (err_code == 0) return true;
  return false;
}

bool setENABLE(uint8_t r)
{
// Set value to ENABLE register
  uint8_t err_code = APDS9960_WRITE(ENABLE, (uint8_t*) &r, 1);
  if (err_code == 0) return true;
  return false;
}

int readGesture() {
  int gesture = _detectedGesture;
  _detectedGesture = GESTURE_NONE;
  return gesture;
}

int gestureFIFOAvailable(void) {
  uint8_t r;
  if (!getGSTATUS(&r))
    return -1;
  // Fill in the appropriate register
  if ((r & 1) == 0) {
    return -2;
    }
  if (!getGFLVL(&r))
    return -3;

  return (int) r;
}

bool enableGesture(void) {
  uint8_t r;
  if (!getENABLE(&r))
    return false;
  // Fill in the appropriate register
  if ((r & 0b1000101) != 0) {
    _gestureEnabled = true;
    return true;
  }
  // Fill in the appropriate register
  r |= ENABLE_GESTURE_START;
  bool res = setENABLE(r);
  _gestureEnabled = res;
  return res;
}

uint8_t readGFIFO_U(uint8_t *fifo_data, int byte_available)
{
  uint8_t err_code = APDS9960_READ(GFIFO_U, (uint8_t *) fifo_data, byte_available);
  if (err_code != 0) return 0xFF;
}

int handleGesture() {
  // You may change the threshold for better use of gesture sensing
  const int gestureThreshold = 30;
  // Check if FIFO data is ready to read
  int available = gestureFIFOAvailable();


  // Read the gesture data in sensor and store to fifo_data
  if (available <= 0)
    return 0;

  uint8_t fifo_data[128];
  uint8_t bytes_read = readGFIFO_U(fifo_data, available * 4);

  // Process the gesture data and indicate the motion
  for (int i = 0; i + 3 < available * 4; i += 4) {
    uint8_t u, d, l, r;
    u = fifo_data[i];
    d = fifo_data[i + 1];
    l = fifo_data[i + 2];
    r = fifo_data[i + 3];
    NRF_LOG_INFO("%x,%x,%x,%x", u, d, l, r);

    if ((u < gestureThreshold) && (d < gestureThreshold) && (l < gestureThreshold) && (r < gestureThreshold)) {
      _gestureIn = true;
      if (_gestureDirInX != 0 || _gestureDirInY != 0) {
        int totalX = _gestureDirInX - _gestureDirectionX;
        int totalY = _gestureDirInY - _gestureDirectionY;
        NRF_LOG_INFO("OUT %d, %d", totalX, totalY);

        if (totalX < -_gestureSensitivity) {
          _detectedGesture = GESTURE_LEFT;
        }
        if (totalX > _gestureSensitivity) {
          _detectedGesture = GESTURE_RIGHT;
        }

        if (totalY < -_gestureSensitivity) {
          _detectedGesture = GESTURE_DOWN;
        }
        if (totalY > _gestureSensitivity) {
          _detectedGesture = GESTURE_UP;
        }

        _gestureDirectionX = 0;
        _gestureDirectionY = 0;
        _gestureDirInX = 0;
        _gestureDirInY = 0;
      }
      continue;
    }

    _gestureDirectionX = r - l;
    _gestureDirectionY = u - d;
    if (_gestureIn) {
      _gestureIn = false;
      _gestureDirInX = _gestureDirectionX;
      _gestureDirInY = _gestureDirectionY;
    }
  }
}

uint8_t gesture_init()
{
  // Pulse length and pulse count init
  uint8_t r = GPULSE_DEFAULT;
  int8_t err_code = APDS9960_WRITE(GPULSE, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);

  // Set GFIFOTH level
  err_code = APDS9960_READ(GCONF1, (uint8_t*) &r, 1);
  r |= GCONF1_DEFAULT;
  err_code = APDS9960_WRITE(GCONF1, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);

  // Set proximity gain, LED drive strength and waitime between detection cycles
  r = GCONF2_DEFAULT;
  err_code = APDS9960_WRITE(GCONF2, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);

  // Enable both sensing dimensions
  err_code = APDS9960_READ(GCONF3, (uint8_t*) &r, 1);
  r &= ~(0b00000011);
  APP_ERROR_CHECK(err_code);
  err_code = APDS9960_WRITE(GCONF3, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);

  // Set GMODE to 0
  err_code = APDS9960_READ(GCONF4, (uint8_t*) &r, 1);
  if (err_code != 0) return 0;
  r &= ~(0b00000001);
  err_code = APDS9960_WRITE(GCONF4, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  return 0b00000001;
}


int main(void) {
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  uint32_t time_sleep = 5000;
  bool detected_device = false;
  nrf_gpio_pin_set(PIN_VDD_ENV);
  nrf_gpio_pin_set(PIN_R_PULLUP);

  nrf_gpio_cfg(PIN_VDD_ENV, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_gpio_cfg(PIN_R_PULLUP, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
      NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
  nrf_delay_ms(4);

  APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
  NRF_LOG_DEFAULT_BACKENDS_INIT();

  NRF_LOG_INFO("TWI scanner started.");
  NRF_LOG_FLUSH();
  twi_init_sensor();
  twi_init_oled();
  OLED_INIT(&m_twi_oled);
  OLED_LOGO(&m_twi_oled);
  /**********************************************/
  /* All the initialization should be done here */
  // Please read the lab document carefully!
  bsp_board_init(BSP_INIT_LEDS);
  bsp_board_leds_off();
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, LED_SLEEP_HANDLER);
  APP_ERROR_CHECK(err_code);

  uint32_t time_ticks_sleep = nrf_drv_timer_ms_to_ticks(&m_timer, time_sleep);
  nrf_drv_timer_compare(&m_timer, NRF_TIMER_CC_CHANNEL0, time_ticks_sleep, false);
  nrf_drv_timer_enable(&m_timer);
  // Pulse length and pulse count init
  gesture_init();
  

  /**********************************************/
  if (!_gestureEnabled) {      
    if (!enableGesture()) {
      NRF_LOG_INFO("Cannot enable the gesture sensor.");
      NRF_LOG_FLUSH();
    }
    // Set GMODE to 1 and start the sensor
    uint8_t r1 = 0b00000001;
    err_code = APDS9960_WRITE(GCONF4, (uint8_t*) &r1, 1);
    APP_ERROR_CHECK(err_code);
  } 
  
  while (true) {
    nrf_delay_ms(100);

/*
    // Set the Gesture sensing as FIFO
    if (!_gestureEnabled){ enableGesture();}

	// Set the Gesture sensing as FIFO
    if (gestureFIFOAvailable() > 0) {
*/

    if (gestureFIFOAvailable() > 0) {
      handleGesture();

      /******************************************/
      /* Control the LEDs from detected gesture */
      bool oled_success = false;
      bsp_board_leds_off();
      nrf_drv_timer_compare_int_enable(&m_timer, NRF_TIMER_CC_CHANNEL0);
      if (_detectedGesture == GESTURE_UP) {
        // red led, display shows inverted version
        bsp_board_led_on(LED_R);
        oled_success = OLED_INVERT(&m_twi_oled);
        if(!oled_success) abort();
        nrf_delay_ms(250);
        bsp_board_leds_off();
        _detectedGesture = GESTURE_NONE;

      } 
      else if (_detectedGesture == GESTURE_DOWN) {
        // blue led, display shows normal version
        bsp_board_led_on(LED_B);
        oled_success = OLED_NORMAL(&m_twi_oled);
        if(!oled_success) abort();
        nrf_delay_ms(250);
        bsp_board_leds_off();
        _detectedGesture = GESTURE_NONE;

      } 
      else if (_detectedGesture == GESTURE_LEFT) {
        // green led, display shows logo
        bsp_board_led_on(LED_G);
        oled_success = OLED_LOGO(&m_twi_oled);
        if(!oled_success) abort();
        nrf_delay_ms(250);
        bsp_board_leds_off();
        _detectedGesture = GESTURE_NONE;

      } 
      else if (_detectedGesture == GESTURE_RIGHT) {
        // white led, display shows clock
        bsp_board_led_on(LED_R);
        bsp_board_led_on(LED_G);
        bsp_board_led_on(LED_B);
        oled_success = OLED_CLOCK(&m_twi_oled);
        if(!oled_success) abort();
        nrf_delay_ms(250);
        bsp_board_leds_off();
        _detectedGesture = GESTURE_NONE;

      } 
      else {
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;
          continue;
      }

    }
  }
}
