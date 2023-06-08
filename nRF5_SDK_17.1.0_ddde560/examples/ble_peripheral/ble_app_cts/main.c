/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_app_cts_c_main main.c
 * @{
 * @ingroup ble_sdk_app_cts_c
 * @brief Current Time Profile sample application.
 *
 * This file contains the source code for a sample application that uses Current Time Service.
 * This is the client role of the profile, implemented on a peripheral device.
 * When a central device connects, the application will trigger a security procedure (if this is not done
 * by the central side first). Completion of the security procedure will trigger a service
 * discovery. When the Current Time Service and Characteristic have been discovered on the
 * server, pressing button 1 will trigger a read of the current time and print it on the UART.
 *
 */
#include "main.h"

//////////////////////////// INHERIT FROM LAB 3 /////////////////////////////

static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

void OLED_SLEEP_HANDLER()
{
  display_status_old = display_status;
  display_status = 0;
  sleep_timer_flag = 0;
  oled_command(SET_DISPLAY_OFF, &m_twi_oled);
}

void OLED_CLOCK_HANDLER(){
  
  update_digit_array();
  map_array();

  if(display_status == 2) {
    uint8_t data_sig = 0x40;
    ret_code_t err_code;
    nrf_twi_sensor_t* p_instance = &m_twi_oled;
    for (uint8_t m=0; m<8; m++) {
      if(text_array[m] != text_array_old[m])
      {
        oled_command(0x21, p_instance);
        oled_command(0x08+m*14, p_instance);
        oled_command(0x15+m*14, p_instance);
        oled_command(0x22, p_instance);
        oled_command(0x03, p_instance);
        oled_command(0x04, p_instance);

        err_code = nrf_twi_sensor_reg_write(p_instance,
                                            command_slave_addr,
                                            data_sig,
                                            empty_digit,
                                            28);
        nrf_delay_ms(20);
        APP_ERROR_CHECK(err_code);

        err_code = nrf_twi_sensor_reg_write(p_instance,
                                            command_slave_addr,
                                            data_sig,
                                            text_array[m],
                                            28);
        nrf_delay_ms(20);
        APP_ERROR_CHECK(err_code);
        text_array_old[m] = text_array[m];
      }
    }
  }
}
/* The handler of Proximity Alert Function
   The sensor measures proximity when the timer fires
   If the proximity value exceeds threshold, an alert
   will be sent to peer devices by BLE
*/
void PROXIMITY_HANDLER(void)
{
  if(m_is_ias_present)
  {
    uint8_t r;
    ret_code_t err_code = NRF_SUCCESS;
    err_code = APDS9960_READ(PDATA, &r, 1);
    NRF_LOG_INFO("PDATA = %d", r);
    if((r < PROXIMITY_THRESHOLD_MILD)&&(m_is_high_alert_signalled||m_is_mild_alert_signalled))
    {
      err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_NO_ALERT);
      APP_ERROR_CHECK(err_code);
      m_is_mild_alert_signalled = false;
      m_is_high_alert_signalled = false;
      NRF_LOG_INFO("No Alert set.");
    }
    else if((r > PROXIMITY_THRESHOLD_MILD)&&(!m_is_mild_alert_signalled)&&(r < PROXIMITY_THRESHOLD_HIGH))
    {    
      err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_MILD_ALERT);
      APP_ERROR_CHECK(err_code);
      m_is_mild_alert_signalled = true;
      m_is_high_alert_signalled = false;
      NRF_LOG_INFO("Mild Alert set.");
    }
    else if((r > PROXIMITY_THRESHOLD_HIGH)&&(!m_is_high_alert_signalled))
    {    
      err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_HIGH_ALERT);
      APP_ERROR_CHECK(err_code);
      m_is_mild_alert_signalled = false;
      m_is_high_alert_signalled = true;
      NRF_LOG_INFO("High Alert set.");
    }
  }
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


    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
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
    //NRF_LOG_INFO("%x,%x,%x,%x", u, d, l, r);

    if ((u < gestureThreshold) && (d < gestureThreshold) && (l < gestureThreshold) && (r < gestureThreshold)) {
      _gestureIn = true;
      if (_gestureDirInX != 0 || _gestureDirInY != 0) {
        int totalX = _gestureDirInX - _gestureDirectionX;
        int totalY = _gestureDirInY - _gestureDirectionY;
        NRF_LOG_INFO("OUT %d, %d", totalX, totalY);
        sleep_timer_reset();
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

  // Set proximity gain
  r = 0b00000100;
  err_code = APDS9960_WRITE(CONTROL, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  
  // Set Proximity Threshold
  r = 0x00;
  err_code = APDS9960_WRITE(PILT, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);

  r = 0x32;
  err_code = APDS9960_WRITE(PIHT, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  
  // Set Proximity Persistance
  r = 0xA0;
  err_code = APDS9960_WRITE(PERS, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  
  // Clear Proximity Interrupt
  err_code = APDS9960_READ(PICLEAR, &r, 1);
  APP_ERROR_CHECK(err_code);

  // Config GMODE exit
  err_code = APDS9960_READ(GCONF1, (uint8_t*) &r, 1);
  if (err_code != 0) return 0;
  r &= ~(0b011111);
  err_code = APDS9960_WRITE(GCONF1, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  // Set GMODE exit threshold
  r = 0;
  err_code = APDS9960_WRITE(GEXTH, &r, 1);
  // Enable Proximity only mode
  r = ENABLE_PROXIMITY;
  err_code = APDS9960_WRITE(ENABLE, (uint8_t*) &r, 1);
  APP_ERROR_CHECK(err_code);
  return 0b00000001;
  
}
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  GPIO_INT_FLAG = true;
  NRF_LOG_INFO("Proximity interrupt triggered.");    
  
  gesture_flag = true;
  if(display_status == 0) {
    display_status = display_status_old;
    sleep_flag = false;
    if(display_status == 1) OLED_CLOCK(&m_twi_oled);
    nrf_delay_ms(200);
    oled_command(SET_DISPLAY_ON, &m_twi_oled);
    sleep_timer_reset();
  }

  // Set GMODE to 1 and start the sensor
  uint8_t r1 = 0b00000001;
  ret_code_t err_code = APDS9960_WRITE(GCONF4, (uint8_t*) &r1, 1);
  r1 = ENABLE_GESTURE_START;
  err_code = APDS9960_WRITE(ENABLE, &r1, 1);    
  APP_ERROR_CHECK(err_code);
  err_code = APDS9960_READ(AICLEAR, &r1, 1);
  APP_ERROR_CHECK(err_code);
  return;
}
void GPIO_FLAG_HANDLER(void){    
    uint8_t r1;
    ret_code_t err_code = NRF_SUCCESS;
    while (display_status) { 
      nrf_delay_ms(100);
      lmd_process_handler();
      if (gestureFIFOAvailable() > 0) {
        handleGesture();
    
        /******************************************/
        /* Control the LEDs from detected gesture */
        bool oled_success = false;
        bsp_board_leds_off();
        if (_detectedGesture == GESTURE_UP) {
          // red led, display shows inverted version
          bsp_board_led_on(LED_R);
          oled_success = OLED_INVERT(&m_twi_oled);
          if(!oled_success) abort();        
          NRF_LOG_INFO("UP.");
          nrf_delay_ms(250);
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;
        } 
        else if (_detectedGesture == GESTURE_DOWN) {
          // blue led, display shows normal version
          bsp_board_led_on(LED_B);
          oled_success = OLED_NORMAL(&m_twi_oled);
          if(!oled_success) abort();        
          NRF_LOG_INFO("DOWN");
          nrf_delay_ms(250);
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;
        } 
        else if (_detectedGesture == GESTURE_LEFT) {
          // green led, display shows logo
          bsp_board_led_on(LED_G);
          if(display_status != 1){
            oled_success = OLED_LOGO(&m_twi_oled);
            display_status = 1;
            if(!oled_success) abort();
          }
          NRF_LOG_INFO("LEFT");
          nrf_delay_ms(250);
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;

        } 
        else if (_detectedGesture == GESTURE_RIGHT) {
          // white led, display shows clock
          bsp_board_led_on(LED_R);
          bsp_board_led_on(LED_G);
          bsp_board_led_on(LED_B);
          if (display_status != 2) {
            oled_success = OLED_CLOCK(&m_twi_oled);
            display_status = 2;
            if(!oled_success) abort();
          }

        
          NRF_LOG_INFO("Right.");
          nrf_delay_ms(250);
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;

        } 
        else {
          NRF_LOG_INFO("None");
          bsp_board_leds_off();
          _detectedGesture = GESTURE_NONE;
        
        }
      }
    }

    r1 = 0b100;
    err_code = APDS9960_WRITE(GCONF4, &r1, 1);
    APP_ERROR_CHECK(err_code);
    
    err_code = APDS9960_READ(AICLEAR, &r1, 1);
    r1 = ENABLE_PROXIMITY;
    err_code = APDS9960_WRITE(ENABLE, &r1, 1);      // Turn off Gesture engine
    GPIO_INT_FLAG = false;
    APP_ERROR_CHECK(err_code);
}

void sleep_timer_reset(void)
{
  ret_code_t err_code;
  if(sleep_timer_flag)
  {
      err_code = app_timer_stop(m_timer_sleep);
      APP_ERROR_CHECK(err_code);
      err_code = app_timer_start(m_timer_sleep, APP_TIMER_TICKS(5000), &OLED_SLEEP_HANDLER);
      APP_ERROR_CHECK(err_code);
  }
  else
  {
      err_code = app_timer_start(m_timer_sleep, APP_TIMER_TICKS(5000), &OLED_SLEEP_HANDLER);
      sleep_timer_flag = true;     
  }
}

void sleep_timer_stop(void)
{
  ret_code_t err_code;
  if(sleep_timer_flag)
  {
      err_code = app_timer_stop(m_timer_sleep);
      APP_ERROR_CHECK(err_code);
      sleep_timer_flag = false;
  }
  else return;
}

/////////////////////////// END INHERIT FROM LAB 3 //////////////////////////


uint32_t ble_cts_c_notif_enable(ble_cts_c_t * p_cts);
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num  Line number of the failing ASSERT call.
 * @param[in] file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
    pm_peer_id_t peer_id;
    uint32_t     peers_to_copy;

    peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
                     *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
    *p_size = 0;

    while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
    {
        p_peers[(*p_size)++] = peer_id;
        peer_id = pm_next_peer_id_get(peer_id);
    }
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    //NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t ret;

        memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
        m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

        peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

        ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
        APP_ERROR_CHECK(ret);

        // Setup the device identies list.
        // Some SoftDevices do not support this feature.
        ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
        if (ret != NRF_ERROR_NOT_SUPPORTED)
        {
            APP_ERROR_CHECK(ret);
        }

        ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            m_peer_id = p_evt->peer_id;

            // Discover peer's services.
            err_code  = ble_db_discovery_start(&m_ble_db_discovery, p_evt->conn_handle);
            APP_ERROR_CHECK(err_code);
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            advertising_start(false);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        {
            // Note: You should check on what kind of white list policy your application should use.
            if (     p_evt->params.peer_data_update_succeeded.flash_changed
                 && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
            {
                NRF_LOG_DEBUG("New Bond, add the peer to the whitelist if possible");
                NRF_LOG_DEBUG("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                               m_whitelist_peer_cnt + 1,
                               BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

                if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
                {
                    // Bonded to a new peer, add it to the whitelist.
                    m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

                    // The whitelist has been modified, update it in the Peer Manager.
                    err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    if (err_code != NRF_ERROR_NOT_SUPPORTED)
                    {
                        APP_ERROR_CHECK(err_code);
                    }

                    err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break;

        default:
            break;
    }
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in] p_evt  Event received from the Current Time Service client.
 */
static void current_time_print(ble_cts_c_evt_t * p_evt)
{
    NRF_LOG_INFO("\r\nCurrent Time:");
    NRF_LOG_INFO("\r\nDate:");

    NRF_LOG_INFO("\tDay of week   %s", (uint32_t)day_of_week[p_evt->
                                                         params.
                                                         current_time.
                                                         exact_time_256.
                                                         day_date_time.
                                                         day_of_week]);

    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.day == 0)
    {
        NRF_LOG_INFO("\tDay of month  Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tDay of month  %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.day);
    }

    NRF_LOG_INFO("\tMonth of year %s",
    (uint32_t)month_of_year[p_evt->params.current_time.exact_time_256.day_date_time.date_time.month]);
    if (p_evt->params.current_time.exact_time_256.day_date_time.date_time.year == 0)
    {
        NRF_LOG_INFO("\tYear          Unknown");
    }
    else
    {
        NRF_LOG_INFO("\tYear          %i",
                       p_evt->params.current_time.exact_time_256.day_date_time.date_time.year);
    }
    NRF_LOG_INFO("\r\nTime:");
    NRF_LOG_INFO("\tHours     %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours);
    NRF_LOG_INFO("\tMinutes   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes);
    NRF_LOG_INFO("\tSeconds   %i",
                   p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds);
    NRF_LOG_INFO("\tFractions %i/256 of a second",
                   p_evt->params.current_time.exact_time_256.fractions256);

    NRF_LOG_INFO("\r\nAdjust reason:\r");
    NRF_LOG_INFO("\tDaylight savings %x",
                   p_evt->params.current_time.adjust_reason.change_of_daylight_savings_time);
    NRF_LOG_INFO("\tTime zone        %x",
                   p_evt->params.current_time.adjust_reason.change_of_time_zone);
    NRF_LOG_INFO("\tExternal update  %x",
                   p_evt->params.current_time.adjust_reason.external_reference_time_update);
    NRF_LOG_INFO("\tManual update    %x",
                   p_evt->params.current_time.adjust_reason.manual_time_update);
}

/*
static void current_time_update_clock(ble_cts_c_evt_t * p_evt) {

  uint8_t hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
  uint8_t min = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
  uint8_t sec = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;

  text_array[0] = digit2text_array[digit_array[hour/10]];
  text_array[1] = digit2text_array[digit_array[hour%10]];

  text_array[3] = digit2text_array[digit_array[min/10]];
  text_array[4] = digit2text_array[digit_array[min%10]];

  text_array[6] = digit2text_array[digit_array[sec/10]];
  text_array[7] = digit2text_array[digit_array[sec%10]];

  ret_code_t err_code;

  for (uint8_t m=0; m<8; m++) {
    oled_command(0x21, &m_twi_oled);
    oled_command(0x08+m*14, &m_twi_oled);
    oled_command(0x15+m*14, &m_twi_oled);
    oled_command(0x22, &m_twi_oled);
    oled_command(0x03, &m_twi_oled);
    oled_command(0x04, &m_twi_oled);

    err_code = nrf_twi_sensor_reg_write(&m_twi_oled,
                                        command_slave_addr,
                                        0x40,
                                        empty_digit,
                                        28);
    nrf_delay_ms(20);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_twi_sensor_reg_write(&m_twi_oled,
                                        command_slave_addr,
                                        0x40,
                                        text_array[m],
                                        28);
    nrf_delay_ms(20);
    APP_ERROR_CHECK(err_code);
    text_array_old[m] = text_array[m];
  }

}*/

/**@brief Function for the timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.");
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            ble_cts_c_notif_enable(&m_cts_c);       
            ble_cts_c_current_time_read(p_cts);
            // When the CTS is found and established, the timer source
            // will be replaced from app_timer to CTS notification
            // err_code = app_timer_stop(m_timer_clock);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. ");
            // CTS not found in this case we just disconnect. There is no reason to stay
            // in the connection for this simple app since it all wants is to interact with CT
            if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.");
            hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
            min = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
            sec = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;
            digit_array[0] = hour/10;
            digit_array[1] = hour%10;
            digit_array[2] = min/10;
            digit_array[3] = min%10;
            digit_array[4] = sec/10;
            digit_array[5] = sec%10;
            //current_time_update_clock(p_evt);
            NRF_LOG_INFO("New Time: %d %d: %d %d: %d %d", digit_array[0], digit_array[1], digit_array[2], digit_array[3],digit_array[4],digit_array[5]);
            
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");
            break;

        default:
            break;
    }
}

/**@brief Function for handling IAS Client events.
 *
 * @details This function will be called for all IAS Client events which are passed to the
 *          application.
 *
 * @param[in] p_ias_c  IAS Client structure.
 * @param[in] p_evt    Event received.
 */
static void on_ias_c_evt(ble_ias_c_t * p_ias_c, ble_ias_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_IAS_C_EVT_DISCOVERY_COMPLETE:
            // IAS is found on peer. The Find Me Locator functionality of this app will work.
            err_code = ble_ias_c_handles_assign(&m_ias_c,
                                                p_evt->conn_handle,
                                                p_evt->alert_level.handle_value);
            NRF_LOG_INFO("Immediate Alert Service is found on peer device.");
            APP_ERROR_CHECK(err_code);

            m_is_ias_present = true;
            break;

        case BLE_IAS_C_EVT_DISCOVERY_FAILED:
            // IAS is not found on peer. The Find Me Locator functionality of this app will NOT work.
            NRF_LOG_INFO("Immediate Alert Service is not found on peer device.");
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_IAS_C_EVT_DISCONN_COMPLETE:
            // Disable alert buttons
            m_is_ias_present = false;
            break;

        default:
            break;
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_cts_c_init_t   cts_init = {0};
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_ias_c_init_t ias_c_init_obj;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CTS.
    cts_init.evt_handler   = on_cts_c_evt;
    cts_init.error_handler = service_error_handler;
    cts_init.p_gatt_queue  = &m_ble_gatt_queue;
    err_code               = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Immediate Alert Service.
    memset(&ias_c_init_obj, 0, sizeof(ias_c_init_obj));

    m_is_mild_alert_signalled = false;
    m_is_high_alert_signalled = false;

    ias_c_init_obj.evt_handler   = on_ias_c_evt;
    ias_c_init_obj.error_handler = service_error_handler;
    ias_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;

    err_code = ble_ias_c_init(&m_ias_c, &ias_c_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters module.
 *
 * @details This function will be called for all events in the Connection Parameters module that
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_cur_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for intercepting GATTC and @ref nrf_ble_gq errors.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_cts_c_t * p_cts = (ble_cts_c_t *)p_ctx;

    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_cts->error_handler != NULL)
    {
        p_cts->error_handler(nrf_error);
    }
}
/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Database Discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective service instances.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
    ble_ias_c_on_db_disc_evt(&m_ias_c, p_evt);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for creating a tx message for writing a CCCD.
 */
static uint32_t cccd_configure(ble_cts_c_t const * const p_cts, 
                               bool                      enable)
{
    nrf_ble_gq_req_t cccd_req;
    uint16_t         cccd_val  = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    uint8_t          cccd[2];

    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = (ble_cts_c_t *)p_cts;
    cccd_req.params.gattc_write.handle   = p_cts->char_handles.cts_cccd_handle;;
    cccd_req.params.gattc_write.len      = 2;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(p_cts->p_gatt_queue, &cccd_req, p_cts->conn_handle);
}

uint32_t ble_cts_c_notif_enable(ble_cts_c_t * p_cts)
{
    VERIFY_PARAM_NOT_NULL(p_cts);

    return cccd_configure(p_cts, true);
}
/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("Slow advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("Fast advertising with WhiteList");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("Slow advertising with WhiteList");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    pm_handler_secure_on_connection(p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        // when received notifications from server
        case BLE_GATTC_EVT_HVX:
            //current_time_print(p_evt);
            NRF_LOG_INFO("Notification Received");
            ble_cts_c_current_time_read(&m_cts_c);
            //current_time_update_clock(p_evt);

            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_cur_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_cur_conn_handle);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
            {
                m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_cur_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_cts_c.conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_cts_c_current_time_read(&m_cts_c);
                if (err_code == NRF_ERROR_NOT_FOUND)
                {
                    NRF_LOG_INFO("Current Time Service is not discovered.");
                }
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init()
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type                = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance       = true;
    init.advdata.flags                    = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_solicited.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_solicited.p_uuids  = m_adv_uuids;

    init.config.ble_adv_whitelist_enabled = true;
    init.config.ble_adv_fast_enabled      = true;
    init.config.ble_adv_fast_interval     = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout      = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled      = true;
    init.config.ble_adv_slow_interval     = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout      = APP_ADV_SLOW_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**
 * @brief Database discovery collector initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

//////////////////////////// CUSTOM FUNCTION IMPLEMENTATION /////////////////////////////
/*
 * dev_addr -> Device address
 * reg_adddr -> Register address
 * data -> Buffer which holds data to be written
 * length -> Length of data to be written
 */
int8_t LSM_WRITE(uint8_t reg_addr, uint8_t *data, uint16_t length)
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
    err_code = nrf_drv_twi_tx(&m_twi, LSM_ADDR, buffer, length + 1, false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

   return err_code;
}


int8_t LSM_READ(uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  /* Read the data in APDS-9960 sensor by I2C protocol
   * reg_addr -> Register address
   * data -> Buffer where data read from TWI will be stored
   * length -> Length of data to be read
   */
    uint32_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&m_twi, LSM_ADDR, &reg_addr, 1, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    m_xfer_done = false;
    err_code = nrf_drv_twi_rx(&m_twi, LSM_ADDR, data, length);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);

    return err_code;

}
void LSM_INIT(){
  uint8_t r;
  r = 0b01110000;
  LSM_WRITE(ACC_CTRL, &r, 1);
}

/* Check if accelerometer sensor is available to read or not */
bool LSM_DATA_AVAIL(){
  uint8_t r;
  LSM_READ(LSM_STATUS, &r, 1);
  if(r & 0b00000101){
    return true;
  }
  
  return false;
}

static void LSM_DATA_RX(float *g_x, float *g_y, float *g_z)
{
  uint8_t lsm_raw_data[6];
  if (LSM_DATA_AVAIL()){
    for (int i = 0; i < 6; i++){
      LSM_READ(0x28 + i, &lsm_raw_data[i], 1);
    }
  }
  int x = (lsm_raw_data[1] << 8) + lsm_raw_data[0];
  int y = (lsm_raw_data[3] << 8) + lsm_raw_data[2];
  int z = (lsm_raw_data[5] << 8) + lsm_raw_data[4];
  if(x > 32767)
  {
          x -= 65536;
  }
  if(y > 32767)
  {
          y -= 65536;
  }
  if(z > 32767)
  {
          z -= 65536;
  }
  *g_x = x * 4.0 / 32768.0;
  *g_y = y * 4.0 / 32768.0;
  *g_z = z * 4.0 / 32768.0;
}
void acc_calibrate()
{
  float xsum = 0;
  float ysum = 0;
  float zsum = 0;
  for (int i = 0; i < BUF_SIZE; i++)
  {
    LSM_DATA_RX(&a_x[i], &a_y[i], &a_z[i]);
    xsum += a_x[i];
    ysum += a_y[i];
    zsum += a_z[i];
  }
  xavg = xsum / BUF_SIZE;
  yavg = ysum / BUF_SIZE;
  zavg = zsum / BUF_SIZE;
}
void lmd_process_handler(void)
{
    ret_code_t err_code = NRF_SUCCESS;
    LSM_DATA_RX(&x,&y, &z);
    if(m_is_ias_present)
    {

      if((z > FALL_THRESHOLD)&&(!m_is_high_alert_signalled))
      {    
        err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_HIGH_ALERT);
        APP_ERROR_CHECK(err_code);
        m_is_high_alert_signalled = true;
        NRF_LOG_INFO("High Alert set.");
        nrf_delay_ms(5000);

        err_code = ble_ias_c_send_alert_level(&m_ias_c, BLE_CHAR_ALERT_LEVEL_NO_ALERT);
        APP_ERROR_CHECK(err_code);
        m_is_high_alert_signalled = false;
        NRF_LOG_INFO("No Alert set.");
      }
  
    }
}
/////////////////////////// END CUSTOM FUNCTION IMPLEMENTATION //////////////////////////
/**@brief Function for application main entry.
 */
int main(void)
{
    //////////////////////////// INHERIT FROM LAB 3 /////////////////////////////
    ret_code_t err_code;
    uint8_t address, r1;
    uint8_t sample_data;
    uint32_t time_sleep = 10000;
    bool detected_device = false;

    nrf_gpio_pin_set(PIN_VDD_ENV);
    nrf_gpio_pin_set(PIN_R_PULLUP);

    nrf_gpio_cfg(PIN_VDD_ENV, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(PIN_R_PULLUP, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
    nrf_delay_ms(4);

      NRF_LOG_INFO("TWI scanner started.");
      twi_init_sensor();




      twi_init_oled();
    OLED_INIT(&m_twi_oled);
    map_array();
    OLED_CLOCK(&m_twi_oled);

    /**********************************************/
    /* All the initialization should be done here */
    // Please read the lab document carefully!
    bsp_board_init(BSP_INIT_LEDS);
    bsp_board_leds_off();
    // Pulse length and pulse count init
    lfclk_request();
    gesture_init();
    LSM_INIT();
    acc_calibrate();
    x = y= z = 0;
    timers_init();
    gpio_init();
    sleep_timer_flag = false;
    sleep_flag = false;
    err_code = app_timer_create (&m_timer_clock, APP_TIMER_MODE_REPEATED, &OLED_CLOCK_HANDLER);
    APP_ERROR_CHECK(err_code);
    //err_code = app_timer_create(&m_proximity_timer_id, APP_TIMER_MODE_REPEATED, (void*)&PROXIMITY_HANDLER);
    err_code = app_timer_create(&m_timer_sleep, APP_TIMER_MODE_SINGLE_SHOT, &OLED_SLEEP_HANDLER);
    APP_ERROR_CHECK(err_code);
    app_timer_start(m_timer_clock, APP_TIMER_TICKS(1000), &OLED_CLOCK_HANDLER);
    APP_ERROR_CHECK(err_code);
    //app_timer_start(m_proximity_timer_id, PROXIMITY_MEAS_INTERVAL, &PROXIMITY_HANDLER);
  /////////////////////////// END INHERIT FROM LAB 3 //////////////////////////

  bool erase_bonds;

  // Initialize.
  log_init();
  
  buttons_leds_init(&erase_bonds);
  scheduler_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  db_discovery_init();
  advertising_init();
  peer_manager_init();
  services_init();
  conn_params_init();
  erase_bonds = true;
  // Start execution.
  NRF_LOG_INFO("Current Time service client started.");

  advertising_start(erase_bonds);
  sleep_timer_reset();

  // Enter main loop.
  while (true)
  {
      idle_state_handle();
      //__WFI();  // wait for interrupt for lab 3 functionalities 
      if(GPIO_INT_FLAG) 
      {
        while (display_status) { 
          nrf_delay_ms(100);
          //lmd_process_handler();
          if (gestureFIFOAvailable() > 0) {
            handleGesture();
    
            /******************************************/
            /* Control the LEDs from detected gesture */
            bool oled_success = false;
            bsp_board_leds_off();
            if (_detectedGesture == GESTURE_UP) {
              // red led, display shows inverted version
              bsp_board_led_on(LED_R);
              oled_success = OLED_INVERT(&m_twi_oled);
              if(!oled_success) abort();        
              NRF_LOG_INFO("UP.");
              nrf_delay_ms(250);
              bsp_board_leds_off();
              _detectedGesture = GESTURE_NONE;
              break;
            } 
            else if (_detectedGesture == GESTURE_DOWN) {
              // blue led, display shows normal version
              bsp_board_led_on(LED_B);
              oled_success = OLED_NORMAL(&m_twi_oled);
              if(!oled_success) abort();        
              NRF_LOG_INFO("DOWN");
              nrf_delay_ms(250);
              bsp_board_leds_off();
              _detectedGesture = GESTURE_NONE;
              break;
            } 
            else if (_detectedGesture == GESTURE_LEFT) {
              // green led, display shows logo
              bsp_board_led_on(LED_G);
              if(display_status != 1){
                oled_success = OLED_LOGO(&m_twi_oled);
                display_status = 1;
                if(!oled_success) abort();
              }
              NRF_LOG_INFO("LEFT");
              nrf_delay_ms(250);
              bsp_board_leds_off();
              _detectedGesture = GESTURE_NONE;
              break;

            } 
            else if (_detectedGesture == GESTURE_RIGHT) {
              // white led, display shows clock
              bsp_board_led_on(LED_R);
              bsp_board_led_on(LED_G);
              bsp_board_led_on(LED_B);
              if (display_status != 2) {
                oled_success = OLED_CLOCK(&m_twi_oled);
                display_status = 2;
                if(!oled_success) abort();
              }

        
              NRF_LOG_INFO("Right.");
              nrf_delay_ms(250);
              bsp_board_leds_off();
              _detectedGesture = GESTURE_NONE;
              break;

            } 
            else {
              NRF_LOG_INFO("None");
              bsp_board_leds_off();
              _detectedGesture = GESTURE_NONE;
        
            }
          }
        }

        r1 = 0b100;
        err_code = APDS9960_WRITE(GCONF4, &r1, 1);
        APP_ERROR_CHECK(err_code);
        err_code = APDS9960_READ(AICLEAR, &r1, 1);
        APP_ERROR_CHECK(err_code);

        r1 = ENABLE_PROXIMITY;
        err_code = APDS9960_WRITE(ENABLE, &r1, 1);      // Turn off Gesture engine
        GPIO_INT_FLAG = false;
        APP_ERROR_CHECK(err_code);
      }
      lmd_process_handler();
  }
}


/**
 * @}
 */
