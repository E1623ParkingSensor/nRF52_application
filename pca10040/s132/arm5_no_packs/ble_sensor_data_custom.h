#include "ble.h"
#include "ble_srv_common.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_SDC_SERVICE 0x0001                      /**< The UUID of the SDC Service. */
#define BLE_SDC_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) */



// Sensor Data Custom -> sdc
typedef struct ble_sdc_s ble_sdc_t;
typedef void (*ble_sdc_data_handler_t) (ble_sdc_t * p_sdc, uint8_t * p_data, uint16_t length);


typedef struct
{
    ble_sdc_data_handler_t data_handler; /**< Event handler to be called for handling received data. */
} ble_sdc_init_t;



struct ble_sdc_s
{
    uint8_t                  uuid_type;               /**< UUID type for service Base UUID. */
    uint16_t                 service_handle;          /**< Handle of service (as provided by the SoftDevice). */
    ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
    uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
    ble_sdc_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

/* Connection handler when connecting with service. */
void on_connect(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt);
/* Connection handler when disconnecting from service */
void on_disconnect(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt);
/* Connection handler when receiving write request from applicaton/client */
void on_write(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt);

/* Function for initializing the Send Data Custom service. */
uint32_t ble_sdc_init(ble_sdc_t * p_sdc, const ble_sdc_init_t * p_sdc_init);

/* Handler for Send Data Custom Service on BLE events. Implemented in main.c */
void ble_sdc_on_ble_evt(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt);

/* Function for sending data. */
uint32_t ble_sdc_data_send(ble_sdc_t * p_sdc, uint8_t * p_string, uint16_t length);

/** @} */

