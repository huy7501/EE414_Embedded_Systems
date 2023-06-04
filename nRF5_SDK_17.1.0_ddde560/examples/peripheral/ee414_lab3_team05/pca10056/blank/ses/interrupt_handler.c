void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    // When Interrupt signal is sent by APDS-9960, 
    // the processor sends the command to enable gesture mode
    // and sense the gesture
    uint8_t r1 = 0b00000001;
    err_code = APDS9960_WRITE(GCONF4, (uint8_t*) &r1, 1);
    APP_ERROR_CHECK(err_code);
    while(1)
    {
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