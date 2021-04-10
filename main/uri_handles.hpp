#ifndef URI_HANDLES
#define URI_HANDLES

#include <esp_http_server.h>

/* Our URI handler function to be called during GET /uri request */
esp_err_t get_handler(httpd_req_t *req);

/* URI handler structure for GET /uri */
extern httpd_uri_t uri_get;

esp_err_t post_handler(httpd_req_t *req);
extern httpd_uri_t uri_post;

esp_err_t data_post_handler(httpd_req_t *req);
extern httpd_uri_t uri_data;

esp_err_t jpg_get_image_handler(httpd_req_t *req);
extern httpd_uri_t uri_get_camera;

esp_err_t config_post_handler(httpd_req_t *req);
extern httpd_uri_t uri_post_config;
extern httpd_uri_t uri_options_config;

esp_err_t jpg_get_stream_handler(httpd_req_t *req);
extern httpd_uri_t uri_get_stream;
#endif
