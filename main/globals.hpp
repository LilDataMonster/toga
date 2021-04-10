#ifndef SENSORS
#define SENSORS

#include <sensor.hpp>
#include <vector>
#include <cJSON.h>
#include <ble.hpp>
#include <http_client.hpp>
#include <http_server.hpp>
#include <nvs.hpp>
#include <system.hpp>

// probably should avoid to use extern
#if CONFIG_DHT_SENSOR_ENABLED
#include <dht.hpp>
extern LDM::DHT dht;
#endif

#if CONFIG_BME680_SENSOR_ENABLED
#include <bme680.hpp>
extern LDM::BME680 bme680;
#endif

// initialize vector of sensors
extern std::vector<LDM::Sensor*> sensors;

// global handles
extern cJSON * json_data;
extern LDM::BLE *g_ble;
extern LDM::HTTP_Server *g_http_server;
extern LDM::HTTP_Client *g_http_client;
extern LDM::System *g_system;
extern LDM::NVS *g_nvs;

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

#endif
