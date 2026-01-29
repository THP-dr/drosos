#include "esp_camera.h"
#include <a0912_inferencing.h>
#include "DFRobot_AXP313A.h"
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include "esp_http_server.h"

// 1. FORWARD DECLARATIONS (Fixes the "Invalid Conversion" error)
esp_err_t index_handler(httpd_req_t *req);
esp_err_t stream_handler(httpd_req_t *req);

WiFiMulti wifiMulti;
#define HOSTNAME "esp32-data"

#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    45
#define CAM_PIN_SIOD    -1
#define CAM_PIN_SIOC    -1
#define CAM_PIN_Y9      48
#define CAM_PIN_Y8      46
#define CAM_PIN_Y7      8
#define CAM_PIN_Y6      7
#define CAM_PIN_Y5      4
#define CAM_PIN_Y4      41
#define CAM_PIN_Y3      40
#define CAM_PIN_Y2      39
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    42
#define CAM_PIN_PCLK    5

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

DFRobot_AXP313A Axp; 
camera_fb_t *current_fb = NULL;
SemaphoreHandle_t frame_mutex;

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    if (!current_fb || offset + length > current_fb->len) return -1;
    for (size_t i = 0; i < length; i++) {
      out_ptr[i] = (float)current_fb->buf[offset + i]  / 255.0f  - 1.0f; 
    }
    return 0;
}

void inference_task(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(500))) {
      current_fb = esp_camera_fb_get();
      if (current_fb) {
        // Fixed order for C++20 strictness
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
        signal.get_data = &raw_feature_get_data;
        ei_impulse_result_t result;
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        Serial.print("Running inference... ");

        if (res != EI_IMPULSE_OK ) {
            Serial.printf("ERR: %d\n", res);
        } else {
            Serial.printf("Done (%d ms)\n", result.timing.classification);
            
            // Print FOMO Results
            bool found_something = false;
            for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
                auto bb = result.bounding_boxes[ix];
                if (bb.value > 0.3) { // Threshold
                    Serial.printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", 
                                   bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
                    found_something = true;
                }
            }
            if (!found_something) Serial.println("  No objects detected.");
        }

        esp_camera_fb_return(current_fb);
        current_fb = NULL; 
      }
      xSemaphoreGive(frame_mutex);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// HANDLER IMPLEMENTATIONS
esp_err_t index_handler(httpd_req_t *req) {
    const char* html = "<html><body style='background:#222;color:white;text-align:center;'>"
                       "<h1>AI Stream</h1><img src='/stream' style='width:500px; image-rendering:pixelated;'>"
                       "</body></html>";
    return httpd_resp_send(req, html, strlen(html));
}

esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    char * part_buf = (char *)malloc(128);
    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);

    while (true) {
        if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(100))) {
            fb = esp_camera_fb_get();
            xSemaphoreGive(frame_mutex);
        }
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        uint8_t * _jpg_buf = NULL;
        size_t _jpg_buf_len = 0;
        if (frame2jpg(fb, 15, &_jpg_buf, &_jpg_buf_len)) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
            if (res == ESP_OK) {
                size_t hlen = snprintf(part_buf, 128, _STREAM_PART, _jpg_buf_len);
                res = httpd_resp_send_chunk(req, part_buf, hlen);
            }
            if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
            free(_jpg_buf);
        }
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    free(part_buf);
    return res;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  frame_mutex = xSemaphoreCreateMutex();
  while(Axp.begin() != 0){ delay(500); Serial.println("Camera power init failed!"); }
  Axp.enableCameraPower(Axp.eOV2640);
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_Y2; config.pin_d1 = CAM_PIN_Y3; config.pin_d2 = CAM_PIN_Y4;
  config.pin_d3 = CAM_PIN_Y5; config.pin_d4 = CAM_PIN_Y6; config.pin_d5 = CAM_PIN_Y7;
  config.pin_d6 = CAM_PIN_Y8; config.pin_d7 = CAM_PIN_Y9;
  config.pin_xclk = CAM_PIN_XCLK; config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC; config.pin_href = CAM_PIN_HREF;
  config.sccb_i2c_port = 0;
  config.pin_sccb_sda = CAM_PIN_SIOD; // Updated naming
  config.pin_sccb_scl = CAM_PIN_SIOC; // Updated naming
  config.pin_pwdn = CAM_PIN_PWDN; config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = 10000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_96X96;
  config.fb_count = 2;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  esp_camera_init(&config);

  /*// Apply settings
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 2);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 0);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 0);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 0);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 1200);    // 0 to 1200
  s->set_gain_ctrl(s, 0);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 1);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 0);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 0);        // 0 = disable , 1 = enable
  s->set_lenc(s, 0);           // 0 = disable , 1 = enable
  //s->set_quality(s, 10);
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable*/

  wifiMulti.addAP("COSMOTE-536092-2.4GHz", "vlhelen2003");
  wifiMulti.addAP("PasadesOnly", "AnythingGoes");
  while (wifiMulti.run() != WL_CONNECTED) { delay(500); Serial.println("Wifi init failed!"); }
  MDNS.begin(HOSTNAME);
  httpd_config_t server_config = HTTPD_DEFAULT_CONFIG();
  server_config.stack_size = 8192; 
  httpd_handle_t server = NULL;
  
  if (httpd_start(&server, &server_config) == ESP_OK) {
    // Index Page
    httpd_uri_t index_uri = {}; // Zero-initialize all fields first
    index_uri.uri       = "/";
    index_uri.method    = HTTP_GET;
    index_uri.handler   = index_handler;
    index_uri.user_ctx  = NULL;
    // Stream endpoint
    httpd_uri_t stream_uri = {}; // Zero-initialize all fields first
    stream_uri.uri      = "/stream";
    stream_uri.method   = HTTP_GET;
    stream_uri.handler  = stream_handler;
    stream_uri.user_ctx = NULL;
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &stream_uri);
  }

  xTaskCreatePinnedToCore(inference_task, "Inference", 16384, NULL, 1, NULL, 1);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }