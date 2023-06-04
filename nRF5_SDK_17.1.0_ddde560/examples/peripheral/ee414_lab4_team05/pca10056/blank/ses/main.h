#ifndef MAIN_H
#define MAIN_H

#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_twi_sensor.h"
#include <stdio.h>
#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"


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


// Declaration of instances
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
NRF_TWI_MNGR_DEF(m_twi_oled_mngr, 5, 1);
NRF_TWI_SENSOR_DEF(m_twi_oled, &m_twi_oled_mngr, 128);
APP_TIMER_DEF(m_timer_sleep);
APP_TIMER_DEF(m_timer_clock);

// Declaration of global variables
static volatile bool m_xfer_done = false;
static int display_status = 2; // sleep = 0, logo = 1; clock = 2
static int display_status_old;
static bool sleep_flag;
static bool sleep_timer_flag; // true if sleep timer is running and false otherwise
static bool gesture_flag = false; // true if existing a gesture
// Declaration of function in main.c
static void lfclk_request(void);
void OLED_SLEEP_HANDLER();
void OLED_CLOCK_HANDLER();
void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
static void gpio_init(void);
void twi_handler(nrf_drv_twi_evt_t const *p_event, void *p_context);
void twi_init_sensor(void);
void twi_init_oled(void);


int8_t APDS9960_READ(uint8_t reg_addr, uint8_t *data, uint16_t length);
int8_t APDS9960_WRITE(uint8_t reg_addr, uint8_t *data, uint16_t length);
bool enableGesture(void);
bool getGSTATUS(uint8_t *r);
bool getGFLVL(uint8_t *r);
bool getENABLE(uint8_t *r);
bool setENABLE(uint8_t r);
int readGesture();
int gestureFIFOAvailable(void);
uint8_t readGFIFO_U(uint8_t *fifo_data, int byte_available);
int handleGesture();
uint8_t gesture_init();
void sleep_timer_reset(void);
void sleep_timer_stop(void);
#endif