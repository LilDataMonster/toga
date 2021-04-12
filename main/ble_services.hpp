#ifndef BLE_SERVICE
#define BLE_SERVICE

#include <vector>
#include <globals.hpp>
#include <esp_gatts_api.h>

#define ESP_APP_ID                  0x55
#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define SVC_INST_ID                 0

/* Service */
static const uint32_t GATTS_SERVICE_UUID_LDM       = 0x4C444D00;
static const uint16_t GATTS_CHAR_UUID_MAC          = 0x4D01;
static const uint16_t GATTS_CHAR_UUID_IPV4         = 0x4D02;
#if CONFIG_DHT_SENSOR_ENABLED
static const uint16_t GATTS_CHAR_UUID_DHT          = 0x4D03;
#endif
#if CONFIG_BME680_SENSOR_ENABLED
static const uint16_t GATTS_CHAR_UUID_BME680       = 0x4D04;
static const uint16_t GATTS_NOTIFY_UUID_BME680     = 0x4D05;
#endif

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
esp_err_t bleUpdateIpv4(void);

// GATT server access interface (only one currently for GATT server)
static esp_gatt_if_t server_gatts_if;

#if CONFIG_DHT_SENSOR_ENABLED
esp_err_t bleUpdateDht(void);
extern float dht_data[2];
#endif

#if CONFIG_BME680_SENSOR_ENABLED
esp_err_t bleUpdateBme680(void);
extern float bme680_data[4];
extern uint8_t bme_notify;
#endif

/* Attributes State Machine */
enum {
    LDM_IDX_SVC,

    LDM_MAC_CHAR,
    LDM_MAC_VAL,

    LDM_IPV4_CHAR,
    LDM_IPV4_VAL,

#if CONFIG_DHT_SENSOR_ENABLED
    LDM_DHT_CHAR,
    LDM_DHT_VAL,
#endif

#if CONFIG_BME680_SENSOR_ENABLED
    LDM_BME680_CHAR,
    LDM_BME680_VAL,
    LDM_BME680_CFG,
    // LDM_BME680_NOTIFY_VAL,
#endif

    LDM_IDX_NB
};

#endif
