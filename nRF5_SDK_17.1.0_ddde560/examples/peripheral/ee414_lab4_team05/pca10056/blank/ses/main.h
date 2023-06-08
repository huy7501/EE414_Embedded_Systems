#ifndef MAIN_H
#define MAIN_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "ble.h"
#include "ble_cts_c.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble_hci.h"
#include "ble_gap.h"
#include "ble_err.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "nrf_sdh.h"
#include "nrf_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_srv_common.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_ans_c.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "peer_data_storage.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


#include "app_util_platform.h"
#include "boards.h"
#include "nrf_drv_twi.h"
#include "nrf_twi_sensor.h"
#include <stdio.h>
#include "nrf_drv_gpiote.h"
#include "nrfx_gpiote.h"
#include "nrf_drv_clock.h"


#include "oled_display.h"
#include "APDS9960.h"
#include "nrf_delay.h"

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

#define DEVICE_NAME                     "EE414"                       /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_FAST_INTERVAL           40                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_SLOW_INTERVAL           3200                                        /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */

#define APP_ADV_FAST_DURATION           3000                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           18000                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */

#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define MESSAGE_BUFFER_SIZE             18                                          /**< Size of buffer holding optional messages in notifications. */
#define BLE_ANS_NB_OF_CATEGORY_ID       10                                          /**< Number of categories. */

#define SEC_PARAM_TIMEOUT               30  
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE                   /**< Maximum size of scheduler events. Note that scheduler BLE stack events do not contain any data, as the events are being pulled from the stack in the event handler. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


typedef enum
{
    ALERT_NOTIFICATION_DISABLED, /**< Alert Notifications has been disabled. */
    ALERT_NOTIFICATION_ENABLED,  /**< Alert Notifications has been enabled. */
    ALERT_NOTIFICATION_ON,       /**< Alert State is on. */
} ble_ans_c_alert_state_t;



// Declaration of instances
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
NRF_TWI_MNGR_DEF(m_twi_oled_mngr, 5, 1);
NRF_TWI_SENSOR_DEF(m_twi_oled, &m_twi_oled_mngr, 128);
APP_TIMER_DEF(m_timer_sleep);
APP_TIMER_DEF(m_timer_clock);

BLE_CTS_C_DEF(m_cts_c);
BLE_ANS_C_DEF(m_ans_c);                                                             /**< Structure used to identify the Alert Notification Service Client. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);                                           /**< DB discovery module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

// Declaration of global variables
static char const * day_of_week[] =
{
    "Unknown",
    "Monday",
    "Tuesday",
    "Wednesday",
    "Thursday",
    "Friday",
    "Saturday",
    "Sunday"
};

static char const * month_of_year[] =
{
    "Unknown",
    "January",
    "February",
    "March",
    "April",
    "May",
    "June",
    "July",
    "August",
    "September",
    "October",
    "November",
    "December"
};
static pm_peer_id_t m_peer_id;    
static uint16_t m_cur_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static pm_peer_id_t m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];            /**< List of peers currently in the whitelist. */
static uint32_t     m_whitelist_peer_cnt;                                           /**< Number of peers currently in the whitelist. */

static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_CURRENT_TIME_SERVICE, BLE_UUID_TYPE_BLE}};

static uint8_t m_alert_message_buffer[MESSAGE_BUFFER_SIZE];                         /**< Message buffer for optional notify messages. */
static ble_ans_c_alert_state_t m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of New Alert Notifications, i.e. Enabled, Alert On, Disabled. */
static ble_ans_c_alert_state_t m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of Unread Alert Notifications, i.e. Enabled, Alert On, Disabled. */

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