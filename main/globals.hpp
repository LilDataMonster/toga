#ifndef SENSORS
#define SENSORS

#include <sensor.hpp>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string>
#include <vector>
#include <cJSON.h>
#include <ble.hpp>
#include <http_client.hpp>
#include <http_server.hpp>
#include <nvs.hpp>
#include <system.hpp>

// probably should avoid to use extern
#if CONFIG_BME680_SENSOR_ENABLED
#include <bme680.hpp>
extern LDM::BME680 bme680;
#endif

// initialize vector of sensors
extern std::vector<LDM::Sensor*> sensors;

// global handles
extern cJSON * json_data;
extern SemaphoreHandle_t json_mutex;

extern LDM::BLE *g_ble;
extern LDM::HTTP_Server *g_http_server;
extern LDM::HTTP_Client *g_http_client;
extern LDM::System *g_system;
extern LDM::NVS *g_nvs;

extern LDM::WiFi *g_wifi;

extern uint8_t mac[6];
extern uint8_t ipv4[4];

// led parameters
extern uint16_t led_fade_time;
extern uint16_t led_duty;
extern int32_t led_on;
extern uint32_t led_period_ms;
extern bool is_period_enabled;
extern bool is_camera_led_flash_enabled;

// system parameters
extern uint32_t sleep_period_ms;
extern uint32_t read_sensor_period_ms;
extern uint32_t publish_url;

// transmit parameters
extern std::string g_post_url;
extern std::string g_firmware_upgrade_url;

// build vector of transmitters to enable transitions between transmitters
typedef struct{
    uint8_t protocol;
    bool enabled;
    uint32_t duration;
    char* name;
} transmit_t;

enum Protocol : uint8_t { wifi, ble, xbee };
extern std::vector<transmit_t> transmitters;

#endif
