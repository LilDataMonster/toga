#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_pm.h>
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <time.h>
#include <sys/time.h>
#include <tasks.hpp>
#include <vector>
#include <cstring>

#define APP_TAG "TOGA"

#include <nvs.hpp>
#include <sleep.hpp>
#include <system.hpp>
#include <http_server.hpp>
#include <ble.hpp>

#include <uri_handles.hpp>
#include <globals.hpp>

extern "C" {
void app_main(void);
}

#if CONFIG_BME680_SENSOR_ENABLED
#include <bme680.hpp>
LDM::BME680 bme680;
#endif

// define extern variables
LDM::NVS *g_nvs = NULL;
LDM::BLE *g_ble = NULL;
LDM::HTTP_Server *g_http_server = NULL;
LDM::HTTP_Client *g_http_client = NULL;
LDM::System *g_system = NULL;
LDM::WiFi *g_wifi = NULL;

std::vector<LDM::Sensor*> sensors {
#if CONFIG_BME680_SENSOR_ENABLED
    &bme680,
#endif
};

std::vector<transmit_t> transmitters {
    {Protocol::wifi, false, 120000, "wifi"},
    {Protocol::ble,  false, 120000, "ble"},
    // {Protocol::xbee, false, 120000, "xbee"},
};

// define various board modes
enum BoardMode : uint8_t { setup, operational };
// static BoardMode mode = BoardMode::setup;
static BoardMode mode = BoardMode::operational;


uint8_t mac[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ipv4[4] = {0x00, 0x00, 0x00, 0x00};

#define APP_MAIN "LDM:Main"
#define STATUS_GPIO GPIO_NUM_2
#define STATUS_GPIO_SEL GPIO_SEL_2
void app_main(void) {

    esp_err_t err = ESP_OK;

    // report reason for last power-off to console
    LDM::Sleep::getWakeupCause();

    // open the "broadcast" key-value pair from the "state" namespace in NVS
    uint8_t broadcast = 0; // value will default to 0, if not set yet in NVS

    // initialize nvs
    g_nvs = new LDM::NVS();
    g_nvs->openNamespace("system");

    // TODO: Add different modes of which the board can be in
    if(mode == BoardMode::setup) {
        ESP_LOGI(APP_TAG, "Board in setup mode");

        gpio_config_t io_conf;
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = STATUS_GPIO_SEL;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        gpio_config(&io_conf);

        gpio_set_level(STATUS_GPIO, 1);
        xTaskCreate(setup_task, "setup_task", 8192, NULL, 5, NULL);
    }

    // // load/update 'broadcast' variable in NVS
    // g_nvs->getKeyU8("broadcast", &broadcast);
    // broadcast++;
    // g_nvs->setKeyU8("broadcast", broadcast);
    // g_nvs->commit();

    // // initialize wifi configurations
    // wifi_config_t wifi_config = {};
    // size_t wifi_size = 0;
    //
    // uint8_t ssid[32];
    // uint8_t passwd[128];
    //
    // // load wifi settings from NVS memory (or set default if wifi settings don't exist)
    // err = g_nvs->getKeyStr("wifi_ssid", NULL, &wifi_size);      // fetch wifi ssid size (max 32)
    // if(err == ESP_OK) {
    //     // g_nvs->getKeyStr("wifi_ssid", (char*)ssid, &wifi_size);
    //     g_nvs->getKeyStr("wifi_ssid", (char*)wifi_config.sta.ssid, &wifi_size);
    //     g_nvs->getKeyStr("wifi_password", NULL, &wifi_size);  // fetch wifi ssid size (max 64)
    //     // g_nvs->getKeyStr("wifi_password", (char*)passwd, &wifi_size);
    //     g_nvs->getKeyStr("wifi_password", (char*)wifi_config.sta.password, &wifi_size);
    // }
    // g_nvs->close();

    // // setup MAC for broadcasting
    // err = esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // // initialize bluetooth device
    // g_ble = new LDM::BLE(const_cast<char*>(CONFIG_BLUETOOTH_DEVICE_NAME));
    // g_ble->init();                                           // initialize bluetooth
    // g_ble->setupDefaultBlufiCallback();                      // setup blufi configuration
    // g_ble->initBlufi(&wifi_config);                          // initialize blufi with given wifi configuration
    // g_ble->registerGattServerCallback(gatts_event_handler);  // setup ble gatt server callback handle
    // g_ble->registerGattServerAppId(ESP_APP_ID);              // setup ble gatt application profile from database

//     // initialize sensor
//     for(auto const& sensor : sensors) {
//         ESP_LOGI(APP_MAIN, "Initializing Sensor: %s", sensor->getSensorName());
//         sensor->init();
//     }
//
//     // setup http server for modifying devices using REST calls (LEDs, camera, etc.)
//     LDM::HTTP_Server server(const_cast<char*>(""));
//     g_http_server = &server;
//     httpd_config_t * server_config = g_http_server->getConfig();
//     server_config->send_wait_timeout = 20;
//
    // setup RTOS tasks
    ESP_LOGI(APP_MAIN, "Setting up RTOS tasks");
    // xTaskCreate(sleep_task, "sleep_task", configMINIMAL_STACK_SIZE, (void*)&sensors, 5, NULL); // task: watcher for initiating sleeps
    xTaskCreate(transmit_scheduler_task, "transmit_scheduler_task", 8192, NULL, 5, NULL); // task: cycle through timings of different transmitters
    xTaskCreate(sensor_task, "sensor_task", 8192, (void*)&sensors, 5, NULL);                      // task: sensor data
    xTaskCreate(wifi_task, "wifi_task", 8192, NULL, 5, NULL);                                     // task: publishing data with REST POST
    xTaskCreate(ble_task, "ble_task", 8192, NULL, 5, NULL);                                     // task: publishing data with REST POST
//
// #if CONFIG_ZIGBEE_ENABLED
//     xTaskCreate(xbee_task, "xbee_task", 8192, NULL, 5, NULL);
// #endif
//     // xTaskCreate(led_fade_task, "led_task", 3*configMINIMAL_STACK_SIZE, NULL, 5, NULL);         // task: fade LED lights
//     // xTaskCreate(led_on_off_task, "led_task", 3*configMINIMAL_STACK_SIZE, NULL, 5, NULL);          // task: turn on/off LED lights (no fade)
//
//     // sensor task moved here
//     while(true) {
//
//         // update bluetooth characteristic with latest ipv4 address
//         if(ble_dev.wifi.getIpv4(ipv4) != ESP_OK) {
//             ESP_LOGE(APP_MAIN, "Unable to fetch IPV4");
//             ipv4[0] = 0;
//             ipv4[1] = 0;
//             ipv4[2] = 0;
//             ipv4[3] = 0;
//         } else {
//             ESP_LOGI(APP_MAIN, "Fetched IPV4: %x:%x:%x:%x", ipv4[0], ipv4[1], ipv4[2], ipv4[3]);
//             err = bleUpdateIpv4();
//             if(err != ESP_OK) {
//                 ESP_LOGE(APP_MAIN, "Unable to update IPV4 Bluetooth characteristic");
//             }
//         }
//
//         // start onboard HTTP server and register URI target locations for REST handles
//         // TODO: Handle disconnect/stopping server
//         if(g_ble->wifi.isConnected() && !g_http_server->isStarted()) {
//             g_http_server->startServer();
//             if(g_http_server->isStarted()) {
//                 ESP_LOGI(APP_MAIN, "Registering HTTP Server URI Handles");
//                 g_http_server->registerUriHandle(&uri_get);
//                 g_http_server->registerUriHandle(&uri_post);
//                 g_http_server->registerUriHandle(&uri_data);
//                 g_http_server->registerUriHandle(&uri_get_camera);
//                 g_http_server->registerUriHandle(&uri_post_config);
//                 g_http_server->registerUriHandle(&uri_options_config);
//                 g_http_server->registerUriHandle(&uri_get_stream);
//             }
//         }
//
//         // create a fresh JSON buffer (delete/free existing buffer if one already exists)
//         if(json_data != NULL) {
//             cJSON_Delete(json_data);
//             json_data = NULL;
//         }
//         json_data = cJSON_CreateObject();
//
//         // construct JSON data for system information
//         LDM::System system;
//         cJSON *system_json = system.buildJson();
//         cJSON_AddItemToObject(json_data, APP_TAG, system_json);
//
//         // loop through each sensor and collect readings
//         for(auto const& sensor : sensors) {
//             ESP_LOGI(APP_MAIN, "Reading Sensor: %s", sensor->getSensorName());
//
//             // read and update sensor data
//             sensor->readSensor();
//             cJSON *sensor_json = sensor->buildJson();
//             cJSON_AddItemToObject(json_data, sensor->getSensorName(), sensor_json);
//             sensor->releaseData();
//         }
//
// #if CONFIG_BME680_SENSOR_ENABLED
//         // update bluetooth output BME680 sensor data
//         err = bleUpdateBme680();
//         if(err != ESP_OK) {
//             ESP_LOGE(APP_MAIN, "Unable to update BME680 Bluetooth characteristic");
//         }
// #endif
//
//         if(g_nvs != NULL) {
//             // get current wifi config
//             wifi_config_t wifi_config_tmp;
//             err |= g_ble->wifi.getConfig(ESP_IF_WIFI_STA, &wifi_config_tmp);
//
//             // update NVS memory with ssid and password if different from initial ssid/password
//             // this allows for the device to remember WIFI/PASSWORD configurations if power is disconnected
//
//             //ESP_LOGI(APP_MAIN, "Comparing SSID: %s to tmp_SSID: %s, strcmp: %d", wifi_config.sta.ssid, wifi_config_tmp.sta.ssid, std::strcmp((char*)wifi_config.sta.ssid, (char*)wifi_config_tmp.sta.ssid));
//             //ESP_LOGI(APP_MAIN, "Comparing password: %s to tmp_password: %s, strcmp: %d", wifi_config.sta.password, wifi_config_tmp.sta.password, std::strcmp((char*)wifi_config.sta.password, (char*)wifi_config_tmp.sta.password));
//             if(std::strcmp((char*)wifi_config.sta.ssid, (char*)wifi_config_tmp.sta.ssid) != 0 ||
//                std::strcmp((char*)wifi_config.sta.password, (char*)wifi_config_tmp.sta.password) != 0){
//                err = g_nvs->openNamespace("system");
//                if(err == ESP_OK) {
//                    err |= g_nvs->setKeyStr("wifi_ssid", reinterpret_cast<char*>(wifi_config_tmp.sta.ssid));
//                    err |= g_nvs->setKeyStr("wifi_password", reinterpret_cast<char*>(wifi_config_tmp.sta.password));
//                    err |= g_nvs->commit();
//                    g_nvs->close();
//
//                    ESP_LOGI(APP_MAIN, "Updated wifi settings in NVS");
//                    std::strcpy((char*)wifi_config.sta.ssid, (char*)wifi_config_tmp.sta.ssid);
//                    std::strcpy((char*)wifi_config.sta.password, (char*)wifi_config_tmp.sta.password);
//                }
//                if(err != ESP_OK) {
//                    ESP_LOGE(APP_MAIN, "Error saving wifi parameters to NVS");
//                }
//             }
//         }
//
//         // sleep
//         vTaskDelay(pdMS_TO_TICKS(10000));
//     }
}
