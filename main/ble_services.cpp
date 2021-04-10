#include <esp_gatts_api.h>
#include <esp_log.h>
#include <globals.hpp>
#include <ble_services.hpp>

#define BLE_SERVICE_TAG "BLE:Service"

// define memory locations for ble data
float dht_data[2] = {0.0f, 0.0f};
float bme680_data[4] = {0.0f, 0.0f, 0.0f, 0.0f};

uint32_t gatt_handle_table[LDM_IDX_NB];

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/
#define GATTS_LDM_CHAR_VAL_LEN_MAX 500
// #define PREPARE_BUF_MAX_SIZE        1024
// #define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

/* Full Database Description - Used to add attributes into the database */
const esp_gatts_attr_db_t gatt_db[LDM_IDX_NB] = {
    // LDM Service Declaration
    [LDM_IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_LDM), (uint8_t *)&GATTS_SERVICE_UUID_LDM}},

//-----------------------

    /* Mac Address Characteristic Declaration */
    [LDM_MAC_CHAR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read}},

    /* Mac Address Characteristic Value */
    [LDM_MAC_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_MAC, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_LDM_CHAR_VAL_LEN_MAX, sizeof(mac), (uint8_t *)mac}},

//-----------------------

    /* IPv4 Address Characteristic Declaration */
    [LDM_IPV4_CHAR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read}},

    /* IPv4 Address Characteristic Value */
    [LDM_IPV4_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_IPV4, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_LDM_CHAR_VAL_LEN_MAX, sizeof(ipv4), (uint8_t *)&ipv4}},

//-----------------------
#if CONFIG_DHT_SENSOR_ENABLED
    /* DHT Sensor Characteristic Declaration */
    [LDM_DHT_CHAR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read}},

    /* DHT Sensor Characteristic Value */
    [LDM_DHT_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_DHT, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_LDM_CHAR_VAL_LEN_MAX, sizeof(dht_data), (uint8_t *)&dht_data}},
#endif

//-----------------------
#if CONFIG_BME680_SENSOR_ENABLED
    /* BME680 Sensor Characteristic Declaration */
    [LDM_BME680_CHAR]      =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      sizeof(uint8_t), sizeof(uint8_t), (uint8_t *)&char_prop_read}},

    /* BME680 Sensor Characteristic Value */
    [LDM_BME680_VAL]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_BME680, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
    GATTS_LDM_CHAR_VAL_LEN_MAX, sizeof(bme680_data), (uint8_t *)&bme680_data}},
#endif

};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

esp_err_t bleUpdateIpv4(void) {
    // update bluetooth values for ipv4 address
    esp_err_t err = esp_ble_gatts_set_attr_value(gatt_handle_table[2], sizeof(ipv4), ipv4);
    if(err != ESP_OK) {
        ESP_LOGE(BLE_SERVICE_TAG, "Failed to send GATTS Attribute on handle %d : %s",
                                  gatt_handle_table[2], esp_err_to_name(err));
    }
    return err;
}

#if CONFIG_DHT_SENSOR_ENABLED
esp_err_t bleUpdateDht(void) {
    // populate array of data to form ble packet
    dht_data[0] = dht.getHumidity();
    dht_data[1] = dht.getTemperature();

    // update bluetooth values for dht sensor data
    esp_err_t err = esp_ble_gatts_set_attr_value(gatt_handle_table[3], sizeof(dht_data), (uint8_t*)dht_data);
    if(err != ESP_OK) {
        ESP_LOGE(BLE_SERVICE_TAG, "Failed to send GATTS Attribute on handle %d : %s",
                                  gatt_handle_table[3], esp_err_to_name(err));
    }
    return err;
}
#endif

#if CONFIG_BME680_SENSOR_ENABLED

// update index to be after DHT index if it is also present
#if CONFIG_DHT_SENSOR_ENABLED
#define BME680_HANDLE_INDEX 4
#else
#define BME680_HANDLE_INDEX 3
#endif

esp_err_t bleUpdateBme680(void) {
    // populate array of data to form ble packet
    bme680_data[0] = bme680.getHumidity();
    bme680_data[1] = bme680.getTemperature();
    bme680_data[2] = bme680.getPressure();
    bme680_data[3] = bme680.getGas();

    // update bluetooth values for bme680 sensor data
    esp_err_t err = esp_ble_gatts_set_attr_value(gatt_handle_table[BME680_HANDLE_INDEX], sizeof(bme680_data), (uint8_t*)bme680_data);
    if(err != ESP_OK) {
        ESP_LOGE(BLE_SERVICE_TAG, "Failed to send GATTS Attribute on handle %d : %s",
                                  gatt_handle_table[BME680_HANDLE_INDEX], esp_err_to_name(err));
    }
    return err;
}
#endif

void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);


void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, LDM_IDX_NB, SVC_INST_ID);
            if(create_attr_ret != ESP_OK) {
                ESP_LOGE(BLE_SERVICE_TAG, "Create attr table failed, error %s", esp_err_to_name(create_attr_ret));
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_READ_EVT, conn_id %d, trans_id %d, handle %d\n",
                      param->read.conn_id, param->read.trans_id, param->read.handle);
       	    break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_WRITE_EVT, conn_id %d, trans_id %d, handle %d\n",
                      param->write.conn_id, param->write.trans_id, param->write.handle);
            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            // example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            // esp_log_buffer_hex(BLE_SERVICE_TAG, param->connect.remote_bda, 6);
            // esp_ble_conn_update_params_t conn_params = {0};
            // memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            // /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            // conn_params.latency = 0;
            // conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            // conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            // conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            // //start sent the update connection parameters to the peer device.
            // esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(BLE_SERVICE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            // esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            ESP_LOGI(BLE_SERVICE_TAG, "The number handle =%x\n", param->add_attr_tab.num_handle);
            if(param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(BLE_SERVICE_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if(param->add_attr_tab.num_handle != LDM_IDX_NB){
                ESP_LOGE(BLE_SERVICE_TAG, "Create attribute table abnormally, num_handle (%d) \
                        doesn't equal to LDM_IDX_NB(%d)", param->add_attr_tab.num_handle, LDM_IDX_NB);
            }
            else {
                ESP_LOGI(BLE_SERVICE_TAG, "Create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(gatt_handle_table, param->add_attr_tab.handles, sizeof(gatt_handle_table));
                esp_err_t err = esp_ble_gatts_start_service(gatt_handle_table[LDM_IDX_SVC]);
                if(err != ESP_OK) {
                    ESP_LOGE(BLE_SERVICE_TAG, "Failed to start GATTS Service");
                }
                ESP_LOGI(BLE_SERVICE_TAG, "Started GATTS Service");
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
struct gatts_profile_inst gatt_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};


void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if(param->reg.status == ESP_GATT_OK) {
            gatt_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(BLE_SERVICE_TAG, "Reg app failed, app_id %04x, status %d",
                    param->reg.app_id, param->reg.status);
            return;
        }
    }
    do {
        for(int idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if(gatts_if == ESP_GATT_IF_NONE || gatts_if == gatt_profile_tab[idx].gatts_if) {
                if(gatt_profile_tab[idx].gatts_cb) {
                    gatt_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}
