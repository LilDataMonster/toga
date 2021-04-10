#include <esp_event.h>
#include <esp_sleep.h>
#include <esp_log.h>
#include <esp_system.h>

// JSON formatting
#include <cJSON.h>

// project headers
#include <sensor.hpp>
#include <http_client.hpp>
#include <tasks.hpp>
#include <ble.hpp>
#include <ota.hpp>

#include <sleep.hpp>
#include <camera.hpp>
#include <cstring>
#include <vector>

#include <led.hpp>
#include <ble.hpp>
#include <system.hpp>

#include <driver/uart.h>
#include <driver/gpio.h>
#include <uri_handles.hpp>

#include <globals.hpp>


#define SENSOR_TASK_LOG "SENSOR_TASK"
void sensor_task(void *pvParameters) {

    // recast array to vector of sensors
    std::vector<LDM::Sensor*> const *sensors = reinterpret_cast<std::vector<LDM::Sensor*> const*>(pvParameters);

    // setup sensors
    for(auto const& sensor : *sensors) {
        ESP_LOGI(SENSOR_TASK_LOG, "Initializing Sensor: %s", sensor->getSensorName());
        do {
            // initialize sensor
            esp_err_t init_err = sensor->init();

            // retry initialization if failed
            if(init_err != ESP_OK) {
                ESP_LOGE(SENSOR_TASK_LOG, "Error Initializing Sensor: %s, Retrying...", sensor->getSensorName());
                vTaskDelay(pdMS_TO_TICKS(100));  // wait before retrying
            }
        }
        while(sensor->initialized != ESP_OK);
    }

    // read sensors
    while(true){
        // create a fresh JSON buffer (delete/free existing buffer if one already exists)
        if(json_data != NULL) {
            cJSON_Delete(json_data);
            json_data = NULL;
        }
        json_data = cJSON_CreateObject();

        // construct JSON data for system information
        LDM::System system;
        cJSON *system_json = system.buildJson();
        cJSON_AddItemToObject(json_data, "board", system_json);

        // loop through sensors and collect sensor data
        for(auto const& sensor : *sensors) {
            ESP_LOGI(SENSOR_TASK_LOG, "Reading Sensor: %s", sensor->getSensorName());

            // read sensor
            sensor->readSensor();

            // create JSON data containing sensor information
            cJSON *sensor_json = sensor->buildJson();
            cJSON_AddItemToObject(json_data, sensor->getSensorName(), sensor_json);
            sensor->releaseData();

            // char* sensor_out = cJSON_Print(sensor_json);
            // printf("%s\n", sensor_out);
            // free(sensor_out);
        }

        // If you read the sensor data too often, it will heat up
        // http://www.kandrsmith.org/RJS/Misc/Hygrometers/dht_sht_how_fast.html
        // vTaskDelay(2000 / portTICK_PERIOD_MS);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}




/***********************************************************************************/

#define SLEEP_DURATION CONFIG_SLEEP_DURATION
#define BLE_ADVERTISE_DURATION CONFIG_BLE_ADVERTISE_DURATION

cJSON* json_data = NULL;

// sleep task will go to sleep when messageFinished is true
static bool messageFinished = false;

uint16_t led_fade_time = 3000;
uint16_t led_duty = 4000;
// int32_t led_on = 0;

#define GPIO_OUTPUT_IO_0    LED_GPIO
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)
// #define GPIO_OUTPUT_IO_1    19
// #define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
void led_on_off_task(void *pvParameters) {

    // setup LED configuration via GPIO pin
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE; //disable interrupt
    io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; //disable pull-down mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; //disable pull-up mode
    gpio_config(&io_conf); //configure GPIO with the given settings

    while(true) {
        // set GPIO level
        gpio_set_level(LED_GPIO, led_on%2);

        // print status
        ESP_LOGI("LDM:LED", "LED: %d, Period Enabled: %s, Period: %u ms",
                  led_on%2, (is_period_enabled?"True":"False"), led_period_ms);

        // bit flip LSB on GPIO pin output (if on/off flashing is enabled)
        led_on += is_period_enabled ? 1 : 0;

        // sleep
        vTaskDelay(pdMS_TO_TICKS(led_period_ms));
    }
}

void led_fade_task(void *pvParameters) {
    // set LED clock
    ledc_timer_config_t ledc_timer;
    ledc_timer.duty_resolution = LEDC_TIMER_13_BIT; // resolution of PWM duty
    ledc_timer.freq_hz = 5000;                      // frequency of PWM signal
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;    // timer mode
    ledc_timer.timer_num = LEDC_TIMER_1;            // timer index
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;             // Auto select the source clock

    // set LED configufation
    ledc_channel_config_t backlight;
    backlight.channel    = LEDC_CHANNEL_0;
    backlight.duty       = 0;
    backlight.gpio_num   = 14;
    backlight.speed_mode = LEDC_LOW_SPEED_MODE;
    backlight.hpoint     = 0;
    backlight.timer_sel  = LEDC_TIMER_1;

    // initialize LED
    LDM::LED led;
    led.configLedTimer(ledc_timer);
    led.addLedChannelConfig(backlight);
    led.init();

    // fade LED at different rates
    while(1) {
        led.fadeLedWithTime(0);
        vTaskDelay(led_fade_time / portTICK_PERIOD_MS);

        led.fadeLedWithTime(0, 0);
        vTaskDelay(led_fade_time / portTICK_PERIOD_MS);

        led.setDuty(0, led_duty);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        led.setDuty(0, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


#define HTTP_POST_ENDPOINT CONFIG_ESP_POST_ENDPOINT
#define HTTP_TASK_LOG "HTTP_TASK"
//#define GPIO_OUTPUT_PIN 13
//#define GPIO_OUTPUT_PIN_SEL  (1ULL << GPIO_OUTPUT_PIN)
#define FIRMWARE_UPGRADE_ENDPOINT CONFIG_FIRMWARE_UPGRADE_ENDPOINT
void http_task(void *pvParameters) {

    // setup http client
    char* endpoint_url = NULL;
    size_t endpoint_url_size = 0;

    // pull HTTP Post destination from NVS memory else use default location (defined by kconfig)
    // check key and get url size if it exists
    if(g_nvs != NULL) {
        g_nvs->openNamespace("url");
        esp_err_t err = g_nvs->getKeyStr("post", NULL, &endpoint_url_size);
        if(err != ESP_OK) {
            endpoint_url = const_cast<char*>(HTTP_POST_ENDPOINT);
            if(err == ESP_ERR_NVS_NOT_FOUND) {
                ESP_LOGI(HTTP_TASK_LOG, "No Post URL found in NVS, setting to default: %s", endpoint_url);
            }
        } else {
            endpoint_url = (char*)malloc(endpoint_url_size);
            err = g_nvs->getKeyStr("post", endpoint_url, &endpoint_url_size);
            if(err != ESP_OK) {
                free(endpoint_url);
                endpoint_url = const_cast<char*>(HTTP_POST_ENDPOINT);
                ESP_LOGI(HTTP_TASK_LOG, "Error fetching Post URL from NVS, setting to default: %s", endpoint_url);
            }
        }
        g_nvs->close();
    } else {
        endpoint_url = const_cast<char*>(HTTP_POST_ENDPOINT);
        ESP_LOGI(HTTP_TASK_LOG, "No NVS found when fetching Post URL from NVS, setting to default: %s", endpoint_url);
    }

    // initialize HTTP client
    LDM::HTTP_Client http(endpoint_url);
    g_http_client = &http;

#ifdef CONFIG_OTA_ENABLED
    // setup ota updater and checkUpdates
    LDM::OTA ota(const_cast<char*>(FIRMWARE_UPGRADE_ENDPOINT));
#endif

    while(true) {
        if(g_ble->wifi.isConnected()) {
            if(json_data != NULL) {

                // POST JSON data
                http.postJSON(json_data);
                // char* post_data = cJSON_Print(json_data);
                // ESP_LOGI(HTTP_TASK_LOG, "%s", post_data);
                // http.postFormattedJSON(post_data);
                // free(post_data);
            } else {
                ESP_LOGI(HTTP_TASK_LOG, "SENSOR_JSON value is NULL");
            }
#ifdef CONFIG_OTA_ENABLED
            // check for OTA updates
            ota.checkUpdates(true);
#endif
        } else {
            ESP_LOGI(HTTP_TASK_LOG, "Wifi is not connected");
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
    // // cleanup JSON message
    // cJSON_Delete(message);
    // message = NULL;

    // vEventGroupDelete(s_wifi_event_group);
    // wifi.deinit_sta();
    http.deinit();

    // messageFinished = true;
    // vTaskDelete(NULL);
}

#define XBEE_TASK_LOG "XBEE_TASK"
const int RX_BUF_SIZE = 1024;
#define TXD_PIN (GPIO_NUM_23)
#define RXD_PIN (GPIO_NUM_22)
void xbee_task(void *pvParameters) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 4, RX_BUF_SIZE * 4, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    const char *TX_TASK_TAG = "TX_TASK";
    vTaskDelay(pdMS_TO_TICKS(30000));
    while(true) {
        if(json_data != NULL) {

            // POST JSON data
            char *post_data = cJSON_Print(json_data);

            const int len = strlen(post_data);
            esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
            const int txBytes = uart_write_bytes(UART_NUM_1, post_data, len);
            ESP_LOGI(TX_TASK_TAG, "Wrote %d bytes", txBytes);
            printf("%s\n", post_data);
        } else {
            ESP_LOGI(XBEE_TASK_LOG, "SENSOR_JSON value is NULL");
        }
        vTaskDelay(pdMS_TO_TICKS(60000));
    }

}

#define SLEEP_TASK_LOG "SLEEP_TASK"
void sleep_task(void *pvParameters) {

    // recast array to vector of sensors
    std::vector<LDM::Sensor*> const *sensors = reinterpret_cast<std::vector<LDM::Sensor*> const*>(pvParameters);

    while(true) {
        // check if ready to go to sleep (aka message is finished sending)
        if(messageFinished) {

            // deinitialize sensors
            for(auto const& sensor : *sensors) {
                sensor->deinit();
            }

            // enter deep sleep
            LDM::Sleep::enterDeepSleepSec(SLEEP_DURATION);
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
