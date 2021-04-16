#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <cJSON.h>
#include <esp_log.h>
#include <camera.hpp>
#include <globals.hpp>
#include <uri_handles.hpp>

#include <string>

#define TAG "URI_HANDLE"

// #define SCRATCH_BUFSIZE (1024)
// typedef struct rest_server_context {
//     // char base_path[ESP_VFS_PATH_MAX + 1];
//     char scratch[SCRATCH_BUFSIZE];
// } rest_server_context_t;

rest_server_context_t *rest_context = (rest_server_context_t*)calloc(1, sizeof(rest_server_context_t));

int32_t led_on = 0;
uint32_t led_period_ms = 10000;
bool is_period_enabled = false;
bool is_camera_led_flash_enabled = false;

typedef struct {
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

esp_err_t setup_base_path(const char* base_path) {
    strlcpy(rest_context->base_path, base_path, sizeof(rest_context->base_path));
    return ESP_OK;
}

// GET URI
httpd_uri_t uri_get = {
    .uri      = "/info",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
};

esp_err_t get_handler(httpd_req_t *req) {
    /* Send a simple response */
    // const char resp[] = "Woot";
    std::string resp = "Woot!\nTemperature is " + std::to_string(bme680.getTemperature()) + "\nHumidity is " + std::to_string(bme680.getHumidity());
    httpd_resp_send(req, resp.c_str(), HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/* URI handler for getting web server files */
httpd_uri_t common_get_uri = {
    .uri = "/*",
    .method = HTTP_GET,
    .handler = rest_common_get_handler,
    .user_ctx = rest_context
};

#define CHECK_FILE_EXTENSION(filename, ext) (strcasecmp(&filename[strlen(filename) - strlen(ext)], ext) == 0)

/* Set HTTP response content type according to file extension */
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filepath) {
    const char *type = "text/plain";
    if(CHECK_FILE_EXTENSION(filepath, ".html")) {
        type = "text/html";
    } else if(CHECK_FILE_EXTENSION(filepath, ".js")) {
        type = "application/javascript";
    } else if(CHECK_FILE_EXTENSION(filepath, ".css")) {
        type = "text/css";
    } else if(CHECK_FILE_EXTENSION(filepath, ".png")) {
        type = "image/png";
    } else if(CHECK_FILE_EXTENSION(filepath, ".ico")) {
        type = "image/x-icon";
    } else if(CHECK_FILE_EXTENSION(filepath, ".svg")) {
        type = "text/xml";
    }
    return httpd_resp_set_type(req, type);
}

#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + 128)
/* Send HTTP response with the contents of the requested file */
esp_err_t rest_common_get_handler(httpd_req_t *req) {
    char filepath[FILE_PATH_MAX];

    rest_server_context_t *rest_context = (rest_server_context_t *)req->user_ctx;
    strlcpy(filepath, rest_context->base_path, sizeof(filepath));
    if(req->uri[strlen(req->uri) - 1] == '/') {
        strlcat(filepath, "/index.html", sizeof(filepath));
    } else {
        strlcat(filepath, req->uri, sizeof(filepath));
    }
    int fd = open(filepath, O_RDONLY, 0);
    if(fd == -1) {
        ESP_LOGE(TAG, "Failed to open file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filepath);

    char *chunk = rest_context->scratch;
    ssize_t read_bytes;
    do {
        /* Read file in chunks into the scratch buffer */
        read_bytes = read(fd, chunk, SCRATCH_BUFSIZE);
        if(read_bytes == -1) {
            ESP_LOGE(TAG, "Failed to read file : %s", filepath);
        } else if(read_bytes > 0) {
            /* Send the buffer contents as HTTP response chunk */
            if(httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) {
                close(fd);
                ESP_LOGE(TAG, "File sending failed!");
                /* Abort sending file */
                httpd_resp_sendstr_chunk(req, NULL);
                /* Respond with 500 Internal Server Error */
                httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
                return ESP_FAIL;
            }
        }
    } while(read_bytes > 0);
    /* Close file after sending complete */
    close(fd);
    ESP_LOGI(TAG, "File sending complete");
    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

// POST URI
httpd_uri_t uri_post = {
    .uri      = "/post",
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx = NULL
};

esp_err_t post_handler(httpd_req_t *req) {
    /* Send a simple response */
    const char resp[] = "Woot";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
};

// Data Post URI
esp_err_t data_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;

    esp_err_t err;
    // size_t buf_len = httpd_req_get_url_query_len(req);
    // char* buffer = (char*)malloc(buf_len);
    // err = httpd_req_get_url_query_str(req, buffer, buf_len);
    // if(err != ESP_OK) {
    //     ESP_LOGE(TAG, "Error getting query string");
    //     return err;
    // }

    const char* field = "Content-Type";
    size_t buf_len = httpd_req_get_hdr_value_len(req, field);
    char* buffer = (char*)malloc(buf_len);
    err = httpd_req_get_hdr_value_str(req, field, buffer, buf_len);
    if(err != ESP_OK) {
        ESP_LOGE(TAG, "Error getting query string: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "URL QUERY STRING: %s", buffer);

    if (total_len >= SCRATCH_BUFSIZE) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    int red = 0;
    int green = 0;
    int blue = 0;

    cJSON *root = cJSON_Parse(buf);
    cJSON *red_json = cJSON_GetObjectItemCaseSensitive(root, "red");
    if(red_json) {
        red = red_json->valueint;
    } else {
        ESP_LOGI(TAG, "Red value not found");
    }
    cJSON *green_json = cJSON_GetObjectItemCaseSensitive(root, "green");
    if(green_json) {
        green = green_json->valueint;
    } else {
        ESP_LOGI(TAG, "Green value not found");
    }
    cJSON *blue_json = cJSON_GetObjectItemCaseSensitive(root, "blue");
    if(blue_json) {
        blue = blue_json->valueint;
    } else {
        ESP_LOGI(TAG, "Blue value not found");
    }
    ESP_LOGI(TAG, "Light control: red = %d, green = %d, blue = %d", red, green, blue);
    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Post control value successfully");
    return ESP_OK;
}

httpd_uri_t uri_data = {
   .uri      = "/data",
   .method   = HTTP_POST,
   .handler  = data_post_handler,
   .user_ctx = rest_context
};

// Camera URI
// static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len) {
//     jpg_chunking_t *j = (jpg_chunking_t *) arg;
//     if(!index) {
//         j->len = 0;
//     }
//     if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
//         return 0;
//     }
//     j->len += len;
//     return len;
// }

esp_err_t jpg_get_image_handler(httpd_req_t *req) {
#if CONFIG_CAMERA_SENSOR_ENABLED
    // camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t fb_len = 0;
    int64_t fr_start = esp_timer_get_time();

    // fb = esp_camera_fb_get();
    // if (!fb) {
    //     ESP_LOGE(TAG, "Camera capture failed");
    //     httpd_resp_send_500(req);
    //     return ESP_FAIL;
    // }
    camera.releaseData();
    res = camera.readSensor();
    if(res != ESP_OK) {
        ESP_LOGE(TAG, "Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    res = httpd_resp_set_type(req, "image/jpeg");
    if(res == ESP_OK){
        res = httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    }

    fb_len = camera.getJpgBufferLength();
    res = httpd_resp_send(req, (const char *)camera.getJpgBuffer(), camera.getJpgBufferLength());

    // fb = camera.getFrameBuffer();
    // jpg_chunking_t jchunk = {req, 0};
    // res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
    // httpd_resp_send_chunk(req, NULL, 0);
    // fb_len = jchunk.len;

    // if(res == ESP_OK){
    //     if(fb->format == PIXFORMAT_JPEG){
    //         fb_len = fb->len;
    //         res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    //     } else {
    //         jpg_chunking_t jchunk = {req, 0};
    //         res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
    //         httpd_resp_send_chunk(req, NULL, 0);
    //         fb_len = jchunk.len;
    //     }
    // }
    // esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    ESP_LOGI(TAG, "JPG: %uKB %ums", (uint32_t)(fb_len/1024), (uint32_t)((fr_end - fr_start)/1000));
    return res;
#else
    return ESP_OK;
#endif
}

httpd_uri_t uri_get_camera = {
   .uri      = "/camera",
   .method   = HTTP_GET,
   .handler  = jpg_get_image_handler,
   .user_ctx = NULL
};

#define GET_INT_VAL_TYPE(_json, _root, _key, _fn, _type)                 \
      cJSON * _json = cJSON_GetObjectItemCaseSensitive(_root, #_key);    \
      if(cJSON_IsNumber(_json)) {                                        \
          s->_fn(s, (_type)_json->valueint);                             \
          ESP_LOGI(TAG, #_key" value set: %d", _json->valueint);         \
      } else {                                                           \
          ESP_LOGI(TAG, #_key" value not found");                        \
      }

#define GET_INT_VAL(_json, _root, _key, _fn) \
      GET_INT_VAL_TYPE(_json, _root, _key, _fn, int)

esp_err_t config_post_handler(httpd_req_t *req) {
    esp_err_t res = ESP_OK;
    int total_len = req->content_len;
    int cur_len = 0;
    char *buf = ((rest_server_context_t *)(req->user_ctx))->scratch;
    int received = 0;
    if (total_len >= SCRATCH_BUFSIZE) {
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "content too long");
        return ESP_FAIL;
    }
    while (cur_len < total_len) {
        received = httpd_req_recv(req, buf + cur_len, total_len);
        if (received <= 0) {
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to post control value");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);

#if CONFIG_CAMERA_SENSOR_ENABLED
    sensor_t *s = camera.getSensor();

    // configure camera
    cJSON *camera_json = cJSON_GetObjectItemCaseSensitive(root, "camera");
    if(cJSON_IsObject(camera_json)) {
        // framesize [0-10]
        GET_INT_VAL_TYPE(framesize_json, camera_json, framesize, set_framesize, framesize_t);
        GET_INT_VAL(quality_json, camera_json, quality, set_quality);
        GET_INT_VAL(contrast_json, camera_json, contrast, set_contrast);
        GET_INT_VAL(brightness_json, camera_json, brightness, set_brightness);
        GET_INT_VAL(saturation_json, camera_json, saturation, set_saturation);
        GET_INT_VAL_TYPE(gainceiling_json, camera_json, gainceiling, set_gainceiling, gainceiling_t);
        GET_INT_VAL(colorbar_json, camera_json, colorbar, set_colorbar);
        GET_INT_VAL(whitebal_json, camera_json, whitebal, set_whitebal);
        GET_INT_VAL(gain_ctrl_json, camera_json, gain_ctrl, set_gain_ctrl);
        GET_INT_VAL(exposure_ctrl_json, camera_json, exposure_ctrl, set_exposure_ctrl);
        GET_INT_VAL(hmirror_json, camera_json, hmirror, set_hmirror);
        GET_INT_VAL(sharpness_json, camera_json, sharpness, set_sharpness);
        GET_INT_VAL(vflip_json, camera_json, vflip, set_vflip);
        GET_INT_VAL(awb_gain_json, camera_json, awb_gain, set_awb_gain);
        GET_INT_VAL(agc_gain_json, camera_json, agc_gain, set_agc_gain);
        GET_INT_VAL(aec_value_json, camera_json, aec_value, set_aec_value);
        GET_INT_VAL(aec2_json, camera_json, aec2, set_aec2);
        GET_INT_VAL(dcw_json, camera_json, dcw, set_dcw);
        GET_INT_VAL(bpc_json, camera_json, bpc, set_bpc);
        GET_INT_VAL(wpc_json, camera_json, wpc, set_wpc);
        GET_INT_VAL(raw_gma_json, camera_json, raw_gma, set_raw_gma);
        GET_INT_VAL(lenc_json, camera_json, lenc, set_lenc);
        GET_INT_VAL(special_effect_json, camera_json, special_effect, set_special_effect);
        GET_INT_VAL(wb_mode_json, camera_json, wb_mode, set_wb_mode);
        GET_INT_VAL(ae_level_json, camera_json, ae_level, set_ae_level);
    } else {
        ESP_LOGI(TAG, "Camera Configuration Not Found");
    }
#endif

    // configure led
    cJSON *led_json = cJSON_GetObjectItemCaseSensitive(root, "led");
    if(cJSON_IsObject(led_json)) {
        cJSON *duty_json = cJSON_GetObjectItemCaseSensitive(led_json, "duty");
        if(cJSON_IsNumber(duty_json)) {
            led_duty = duty_json->valueint;
            ESP_LOGI(TAG, "LED Duty value set: %d", led_duty);
        } else {
            ESP_LOGI(TAG, "duty value not found");
        }
        cJSON *fade_json = cJSON_GetObjectItemCaseSensitive(led_json, "fade");
        if(cJSON_IsNumber(fade_json)) {
            led_fade_time = fade_json->valueint;
            ESP_LOGI(TAG, "LED Fade Time value set: %d", led_fade_time);
        } else {
            ESP_LOGI(TAG, "fade value not found");
        }
        cJSON *on_json = cJSON_GetObjectItemCaseSensitive(led_json, "on");
        if(cJSON_IsBool(on_json)) {
            led_on = cJSON_IsTrue(on_json) ? 1 : 0;
            ESP_LOGI(TAG, "LED On value set: %d", led_on);
        } else {
            ESP_LOGI(TAG, "on value not found");
        }
        cJSON *period_json = cJSON_GetObjectItemCaseSensitive(led_json, "period");
        if(cJSON_IsNumber(period_json)) {
            led_period_ms = period_json->valueint;
            ESP_LOGI(TAG, "LED Period ms value set: %d", led_period_ms);
        } else {
            ESP_LOGI(TAG, "period value not found");
        }
        cJSON *period_en_json = cJSON_GetObjectItemCaseSensitive(led_json, "period_enabled");
        if(cJSON_IsBool(period_en_json)) {
            is_period_enabled = cJSON_IsTrue(period_en_json);
            ESP_LOGI(TAG, "LED Period Enabled value set: %s", (is_period_enabled?"True":"False"));
        } else {
            ESP_LOGI(TAG, "period_enabled value not found");
        }
        cJSON *flash_en_json = cJSON_GetObjectItemCaseSensitive(led_json, "camera_flash");
        if(cJSON_IsBool(flash_en_json)) {
            is_camera_led_flash_enabled = cJSON_IsTrue(flash_en_json);
            ESP_LOGI(TAG, "LED Camera Flash value set: %s", (is_camera_led_flash_enabled?"True":"False"));
        } else {
            ESP_LOGI(TAG, "camera_flash value not found");
        }
    } else {
        ESP_LOGI(TAG, "LED Configuration Not Found");
    }

    // configure board
    cJSON *board_json = cJSON_GetObjectItemCaseSensitive(root, "board");
    if(cJSON_IsObject(board_json)) {
        cJSON *wifi_ms_json = cJSON_GetObjectItemCaseSensitive(board_json, "wifi_duration");
        if(cJSON_IsNumber(wifi_ms_json)) {
            for(auto &transmitter : transmitters) {
                if(transmitter.protocol == Protocol::wifi) {
                    transmitter.duration = wifi_ms_json->valueint;
                }
            }
            ESP_LOGI(TAG, "wifi_ms_json set: %d ms", wifi_ms_json->valueint);
        } else {
            ESP_LOGI(TAG, "wifi_ms_json not found");
        }

        cJSON *ble_ms_json = cJSON_GetObjectItemCaseSensitive(board_json, "ble_duration");
        if(cJSON_IsNumber(ble_ms_json)) {
            for(auto &transmitter : transmitters) {
                if(transmitter.protocol == Protocol::ble) {
                    transmitter.duration = ble_ms_json->valueint;
                }
            }
            ESP_LOGI(TAG, "ble_ms_json set: %d ms", ble_ms_json->valueint);
        } else {
            ESP_LOGI(TAG, "ble_ms_json not found");
        }

        cJSON *xbee_ms_json = cJSON_GetObjectItemCaseSensitive(board_json, "xbee_duration");
        if(cJSON_IsNumber(xbee_ms_json)) {
            for(auto &transmitter : transmitters) {
                if(transmitter.protocol == Protocol::xbee) {
                    transmitter.duration = xbee_ms_json->valueint;
                }
            }
            ESP_LOGI(TAG, "xbee_ms_json set: %d ms", xbee_ms_json->valueint);
        } else {
            ESP_LOGI(TAG, "xbee_ms_json not found");
        }

        // update post_url
        cJSON *post_url_json = cJSON_GetObjectItemCaseSensitive(board_json, "post_url");
        if(cJSON_IsString(post_url_json) && (post_url_json->valuestring != NULL)) {
            // set new URL
            g_post_url = std::string(post_url_json->valuestring);
        }

        // check this last
        cJSON *operational_en_json = cJSON_GetObjectItemCaseSensitive(board_json, "operational");
        if(cJSON_IsBool(operational_en_json)) {
            mode = cJSON_IsTrue(operational_en_json) ? BoardMode::operational : BoardMode::setup;
            ESP_LOGI(TAG, "Exitting Setup Mode: %s", (cJSON_IsTrue(operational_en_json)?"True":"False"));
        } else {
            ESP_LOGI(TAG, "operational not found");
        }
    } else {
        ESP_LOGI(TAG, "Board Configuration Not Found");
    }

    // configure url
    cJSON *report_json = cJSON_GetObjectItemCaseSensitive(root, "report");
    if(cJSON_IsObject(report_json)) {
        cJSON *post_url_json = cJSON_GetObjectItemCaseSensitive(report_json, "post_url");
        if(cJSON_IsString(post_url_json) && (post_url_json->valuestring != NULL)) {
            // set new URL
            g_http_client->setURL(post_url_json->valuestring);

            // update url in NVS
            if(g_nvs != NULL) {
                g_nvs->openNamespace("url");
                esp_err_t err = g_nvs->setKeyStr("post", const_cast<char*>(g_http_client->getURL()));
                if(err != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to update NVS with new url::post string: %s", g_http_client->getURL());
                } else {
                    g_nvs->commit();
                }
                g_nvs->close();
            } else {
                ESP_LOGI(TAG, "No NVS found when updating Post URL: %s", g_http_client->getURL());
            }

            ESP_LOGI(TAG, "Report URL Post value set: %s", post_url_json->valuestring);
        } else {
            ESP_LOGI(TAG, "post_url value not found");
        }
    } else {
        ESP_LOGI(TAG, "Report URL Configuration Not Found");
    }

    cJSON_Delete(root);
    httpd_resp_sendstr(req, "Configuration Updated successfully");

    return res;
};

httpd_uri_t uri_post_config = {
   .uri      = "/config",
   .method   = HTTP_POST,
   .handler  = config_post_handler,
   .user_ctx = rest_context
};

// methods: https://github.com/espressif/esp-idf/blob/master/components/nghttp/port/include/http_parser.h
httpd_uri_t uri_options_config = {
   .uri      = "/config",
   .method   = HTTP_OPTIONS,
   // .handler  = camera_options_handler,
   .handler  = config_post_handler,
   .user_ctx = rest_context
};

// Stream URI
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
esp_err_t jpg_get_stream_handler(httpd_req_t *req) {
#if CONFIG_CAMERA_SENSOR_ENABLED
    // camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len;
    uint8_t * _jpg_buf;
    char * part_buf[64];
    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK) {
        return res;
    }

    while(true){
        camera.releaseData();
        res = camera.readSensor();
        if(res != ESP_OK) {
            ESP_LOGE(TAG, "Camera capture failed");
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }

        _jpg_buf_len = camera.getJpgBufferLength();
        _jpg_buf = camera.getJpgBuffer();
        // if(fb->format != PIXFORMAT_JPEG){
        //     bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        //     if(!jpeg_converted){
        //         ESP_LOGE(TAG, "JPEG compression failed");
        //         esp_camera_fb_return(fb);
        //         res = ESP_FAIL;
        //     }
        // } else {
        //     _jpg_buf_len = fb->len;
        //     _jpg_buf = fb->buf;
        // }

        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);

            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        // if(fb->format != PIXFORMAT_JPEG){
        //     free(_jpg_buf);
        // }
        // esp_camera_fb_return(fb);
        camera.releaseData();
        if(res != ESP_OK){
            break;
        }
        int64_t fr_end = esp_timer_get_time();
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
        ESP_LOGI(TAG, "MJPG: %uKB %ums (%.1ffps)",
            (uint32_t)(_jpg_buf_len/1024),
            (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time);
        vTaskDelay(pdMS_TO_TICKS(30));
    }

    last_frame = 0;
    return res;
#else
    return ESP_OK;
#endif
}

httpd_uri_t uri_get_stream = {
   .uri      = "/stream",
   .method   = HTTP_GET,
   .handler  = jpg_get_stream_handler,
   .user_ctx = NULL
};
