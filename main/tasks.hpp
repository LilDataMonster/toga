#ifndef __TASKS_HPP__
#define __TASKS_HPP__

#include <cJSON.h>

#define LED_GPIO GPIO_NUM_4

extern "C" {
void setup_task(void *pvParameters);
void sensor_task(void *pvParameters);
void ble_task(void *pvParameters);
void transmit_scheduler_task(void *pvParameters);
void wifi_task(void *pvParameters);
void xbee_task(void *pvParameters);
void sleep_task(void *pvParameters);
void led_on_off_task(void *pvParameters);
void led_fade_task(void *pvParameters);

extern cJSON * json_data;

}
#endif
