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

BoardMode mode = BoardMode::setup;
// BoardMode mode = BoardMode::operational;
SemaphoreHandle_t json_mutex = xSemaphoreCreateMutex();

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
    {Protocol::xbee, false, 120000, "xbee"},
};

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

    // setup MAC for broadcasting
    err = esp_read_mac(mac, ESP_MAC_WIFI_STA);

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

    while(mode == BoardMode::setup) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

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

    // setup RTOS tasks
    ESP_LOGI(APP_MAIN, "Setting up RTOS tasks");
    // xTaskCreate(sleep_task, "sleep_task", configMINIMAL_STACK_SIZE, (void*)&sensors, 5, NULL); // task: watcher for initiating sleeps
    xTaskCreate(transmit_scheduler_task, "transmit_scheduler_task", 8192, NULL, 5, NULL); // task: cycle through timings of different transmitters
    xTaskCreate(sensor_task, "sensor_task", 8192, (void*)&sensors, 5, NULL);                      // task: sensor data
    xTaskCreate(wifi_task, "wifi_task", 8192*10, NULL, 5, NULL);                                     // task: publishing data with REST POST
    xTaskCreate(ble_task, "ble_task", 8192, NULL, 5, NULL);                                     // task: publishing data with REST POST

#if CONFIG_ZIGBEE_ENABLED
    xTaskCreate(xbee_task, "xbee_task", 8192, NULL, 5, NULL);
 #endif
//     // xTaskCreate(led_fade_task, "led_task", 3*configMINIMAL_STACK_SIZE, NULL, 5, NULL);         // task: fade LED lights
//     // xTaskCreate(led_on_off_task, "led_task", 3*configMINIMAL_STACK_SIZE, NULL, 5, NULL);          // task: turn on/off LED lights (no fade)

}
