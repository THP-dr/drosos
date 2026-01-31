#include "DFRobot_AXP313A.h"
#include "esp_camera.h"
#include <WebServer.h>        // Standard WebServer library
#include <esp32-hal-i2c.h>    // Low-level I2C to override the camera library
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <a0912_inferencing.h> //AI model
#include "edge-impulse-sdk/tensorflow/lite/c/common.h" //TF lite C API

// Camera i2c adress 0x36   
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    45
#define CAM_PIN_SIOD    -1  //DISABLE I2C PINS  
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

#define HOSTNAME "esp32-data"
WiFiMulti WifiMulti;
camera_fb_t *fb = NULL; // Frame buffer pointer
WebServer Server(80);
DFRobot_AXP313A Axp; 

const int imageWidth = 96;
const int imageHeight = 96;
const int bufferLength = imageWidth * imageHeight;
uint8_t current_raw_image_copy[imageWidth * imageHeight];
SemaphoreHandle_t frame_mutex;

int last_x = -1;
int last_y = -1;
String last_label = "";

// HTML and Javascript for the webpage
void handle_root() {
  String htmlResponse = R"rawliteral(
<!DOCTYPE html>
<html>
<body> 
<body style='background:#222;color:white;text-align:center;'>
<h2>OV2640 Grayscale Stream</h2>
<p><strong>Label: <span id="angleValue">0.00</span></strong></p>
<canvas id="cameraCanvasOriginal" width=")rawliteral" + String(imageWidth) + R"rawliteral(" height=")rawliteral" + String(imageHeight) + R"rawliteral(" style="border:1px solid #000;"></canvas>

<script>
function drawToCanvas(canvasId, byteArray) {
    var canvas = document.getElementById(canvasId);
    var ctx = canvas.getContext('2d');
    var imageData = ctx.createImageData(canvas.width, canvas.height);
    for (var i = 0; i < byteArray.length; i++) {
        var val = byteArray[i];
        imageData.data[i * 4 + 0] = val; 
        imageData.data[i * 4 + 1] = val; 
        imageData.data[i * 4 + 2] = val; 
        imageData.data[i * 4 + 3] = 255; 
    }
    ctx.putImageData(imageData, 0, 0);
}

function fetchImagesAndAngle() {
  // Fetch Image (όπως πριν)
  var xhr1 = new XMLHttpRequest();
  xhr1.open('GET', '/capture', true);
  xhr1.responseType = 'arraybuffer';
  xhr1.onload = function(e) {
    if (xhr1.response) {
      drawToCanvas('cameraCanvasOriginal', new Uint8Array(xhr1.response));
      fetchAIResults(); 
    }
  };
  xhr1.send();
}

function fetchAIResults() {
  fetch('/data').then(response => response.json()).then(data => {
    if (data.x !== -1) {
      var canvas = document.getElementById('cameraCanvasOriginal');
      var ctx = canvas.getContext('2d');
      ctx.fillStyle = "red";
      ctx.beginPath();
      ctx.arc(data.x, data.y, 4, 0, 2 * Math.PI);
      ctx.fill();
      ctx.fillText(data.label, data.x + 10, data.y);
      document.getElementById('angleValue').innerText = data.label + " at " + data.x + "," + data.y;
    } else {
      document.getElementById('angleValue').innerText = "Searching...";
    }
  });
}

// Update all data simultaneously every 300ms
setInterval(fetchImagesAndAngle, 300); 
fetchImagesAndAngle(); // Call immediately on load
</script>
</body>
</html>
)rawliteral";
  Server.send(200, "text/html", htmlResponse);
}

void handle_capture() {
  Server.send_P(200, "application/octet-stream", (const char*)current_raw_image_copy, (size_t)bufferLength);
}

void handle_data() {
  String json = "{";
  json += "\"x\":" + String(last_x) + ",";
  json += "\"y\":" + String(last_y) + ",";
  json += "\"label\":\"" + last_label + "\"";
  json += "}";
  Server.send(200, "application/json", json);
}

static int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        // Correct math for your INT8 model (zero_point -128)
        out_ptr[i] = (float)((int16_t)current_raw_image_copy[offset + i] - 128);
    }
    return 0;
}

void inference_task(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(500))) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // Expected: 96*96 = 9216
        signal.get_data = &raw_feature_get_data; // Applies pixel - 128
        
        ei_impulse_result_t result = {0}; // Initialize result struct

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK) {
          Serial.printf("Inference done (%d ms).\n", result.timing.classification);
          
          bool found_something = false;
          // Iterate through all bounding boxes returned by the post-processor
          for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
            auto bb = result.bounding_boxes[ix];
            // Check if the bounding box is valid and meets the confidence threshold
            if (bb.value > 0.2f) { 
              Serial.printf("  DETECTED: %s (Conf: %.3f) [ x: %u, y: %u, w: %u, h: %u ]\n", 
                                   bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
              last_x = bb.x;
              last_y = bb.y;
              last_label = String(bb.label);

              
              found_something = true;
              // Break after the first detection above the threshold (or remove break to show all)
              break; 
            }
          }
          if (!found_something) {
            Serial.println("  No objects detected above threshold.");  
            last_x = -1; 
            last_y = -1;
            last_label = "Searching..."; 
          }
        } else {
             // Log errors from the classifier
             Serial.printf("run_classifier failed with error: %d\n", res);
             last_x = -1; 
             last_y = -1;
             last_label = "Error"; 
        }
        
        xSemaphoreGive(frame_mutex);
    } else {
         // Log if semaphore couldn't be acquired (e.g., inference taking too long)
         Serial.println("Inference task: Semaphore timeout.");
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Control inference rate
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  while(Axp.begin() != 0){ 
    delay(500); 
    Serial.println("Camera power init failed!"); 
  }
  Axp.enableCameraPower(Axp.eOV2640);

  // Configure camera parameters
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_Y2;
  config.pin_d1 = CAM_PIN_Y3;
  config.pin_d2 = CAM_PIN_Y4;
  config.pin_d3 = CAM_PIN_Y5;
  config.pin_d4 = CAM_PIN_Y6;
  config.pin_d5 = CAM_PIN_Y7;
  config.pin_d6 = CAM_PIN_Y8;
  config.pin_d7 = CAM_PIN_Y9;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.sccb_i2c_port = 0; //IMPORTANT TO LEAVE I2C FOR OTHER SENSORS
  config.pin_sscb_sda = CAM_PIN_SIOD; 
  config.pin_sscb_scl = CAM_PIN_SIOC; 
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000; // 20MHz
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Grayscale 
  config.frame_size = FRAMESIZE_96X96; // 96x96 resolution
  config.fb_count = 1; // Number of frame buffers
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM 
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 

  // Initialize the camera (AFTER THE OTHER I2C SENSORS!!!)
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    Serial.println("Check AXP313A power settings and wiring.");
    return;
  }

  Serial.println("Camera initialized successfully!");

  // Apply settings
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
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
  s->set_quality(s, 10);
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  WifiMulti.addAP("COSMOTE-536092-2.4GHz", "vlhelen2003");
  WifiMulti.addAP("PasadesOnly", "AnythingGoes");

  while (WifiMulti.run() != WL_CONNECTED) { 
    delay(500); 
    Serial.println("Wifi init failed!"); 
  }
  MDNS.begin(HOSTNAME);
  Server.on("/", HTTP_GET, handle_root);
  Server.on("/capture", HTTP_GET, handle_capture);
  Server.on("/data", handle_data);
  Server.begin();

  frame_mutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(inference_task, "Inference", 16384, NULL, 1, NULL, 1);
}

void loop() {
  Server.handleClient(); 

  camera_fb_t * fb = esp_camera_fb_get();
  if (fb) {
    if (fb->len == bufferLength) { // Only copy if it matches 9216
      if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(50))) {
        memcpy(current_raw_image_copy, fb->buf, fb->len);
        xSemaphoreGive(frame_mutex);
      }
    } else {
      Serial.printf("Wrong frame size: %zu\n", fb->len);
    }
    esp_camera_fb_return(fb); 
  }
  delay(1);
}
