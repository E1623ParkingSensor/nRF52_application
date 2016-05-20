

#include "ble_srv_common.h"
#include "ble_sensor_data_custom.h"
#include "sdk_common.h"

//#define BLE_UUID_NUS_TX_CHARACTERISTIC 0x0002                      /**< The UUID of the TX Characteristic. */
#define BLE_UUID_NUS_RX_CHARACTERISTIC 0x0003                      /**< The UUID of the RX Characteristic. */

#define BLE_SDC_MAX_RX_CHAR_LEN        BLE_SDC_MAX_DATA_LEN        /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_SDC_MAX_TX_CHAR_LEN        BLE_SDC_MAX_DATA_LEN        /**< Maximum length of the TX Characteristic (in bytes). */

//#define NUS_BASE_UUID                  {{0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E}}

// New UUID for the SDC service
#define SDC_BASE_UUID                  {{0xEA, 0xBA, 0x6F, 0x60, 0xEC, 0x25, 0x11, 0xE5, 0xA7, 0x61, 0x00, 0x02, 0xA5, 0xD5, 0xC5, 0x1B}}

/* Connection handler when connecting with service. */
void on_connect(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt)
{
    p_sdc->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/* Connection handler when disconnecting from service */
void on_disconnect(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sdc->conn_handle = BLE_CONN_HANDLE_INVALID;
}

/* Connection handler when receiving write request from applicaton/client */
void on_write(ble_sdc_t * p_sdc, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (
        (p_evt_write->handle == p_sdc->rx_handles.cccd_handle)
        &&
        (p_evt_write->len == 2)
       )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_sdc->is_notification_enabled = true;
        }
        else
        {
            p_sdc->is_notification_enabled = false;
        }
    }
    else if (
             (p_evt_write->handle == p_sdc->tx_handles.value_handle)
             &&
             (p_sdc->data_handler != NULL)
            )
    {
        p_sdc->data_handler(p_sdc, p_evt_write->data, p_evt_write->len);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

/* Function for adding write characteristic and attributes to the service */
static uint32_t rx_char_add(ble_sdc_t * p_sdc, const ble_sdc_init_t * p_sdc_init)
{
    /**@snippet [Adding proprietary characteristic to S110 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_sdc->uuid_type;
    ble_uuid.uuid = BLE_UUID_NUS_RX_CHARACTERISTIC;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = BLE_SDC_MAX_RX_CHAR_LEN;

    return sd_ble_gatts_characteristic_add(p_sdc->service_handle,&char_md,&attr_char_value,&p_sdc->rx_handles);

}

/* Function for initializing the Send Data Custom service. */
uint32_t ble_sdc_init(ble_sdc_t * p_sdc, const ble_sdc_init_t * p_sdc_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t sdc_base_uuid = SDC_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_sdc);
    VERIFY_PARAM_NOT_NULL(p_sdc_init);

    // Initialize the service structure.
    p_sdc->conn_handle             = BLE_CONN_HANDLE_INVALID;
    p_sdc->data_handler            = p_sdc_init->data_handler;
    p_sdc->is_notification_enabled = false;

    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&sdc_base_uuid, &p_sdc->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_sdc->uuid_type;
    ble_uuid.uuid = BLE_UUID_SDC_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,&ble_uuid,&p_sdc->service_handle);
    /**@snippet [Adding proprietary Service to S110 SoftDevice] */
    VERIFY_SUCCESS(err_code);

    // Add the RX Characteristic.
    err_code = rx_char_add(p_sdc, p_sdc_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

/* Function for sending data. */ 
uint32_t ble_sdc_data_send(ble_sdc_t * p_sdc, uint8_t * p_string, uint16_t length)
{
    ble_gatts_hvx_params_t hvx_params;

    VERIFY_PARAM_NOT_NULL(p_sdc);

    if ((p_sdc->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_sdc->is_notification_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_SDC_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sdc->rx_handles.value_handle;
    hvx_params.p_data = p_string;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_sdc->conn_handle, &hvx_params);
}
/** @} */
