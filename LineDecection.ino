#include "esp_camera.h"
#include "DFRobot_AXP313A.h"  // Include AXP313A library 
#include <WiFi.h>             // Connect ESP32 to WiFi
#include <ArduinoOTA.h>       // Enable over-the-air updates
#include <WebServer.h>        // Standard WebServer library
#include <esp32-hal-i2c.h>    // Low-level I2C to override the camera library
#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // LCD 

// Camera i2c adress 0x36   
#define CAM_PIN_PWDN    -1
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    45
#define CAM_PIN_SIOD    -1  //DISABLE I2C PINS  //1  // Dedicated I2C SDA for camera sensor
#define CAM_PIN_SIOC    -1                      //2  // Dedicated I2C SCL for camera sensor

#define CAM_PIN_Y9      48
#define CAM_PIN_Y8      46
#define CAM_PIN_Y7      8
#define CAM_PIN_Y6      7 //PROBLEM CONFLICT WITH MOTOR DRIVER
#define CAM_PIN_Y5      4
#define CAM_PIN_Y4      41
#define CAM_PIN_Y3      40
#define CAM_PIN_Y2      39
#define CAM_PIN_VSYNC   6
#define CAM_PIN_HREF    42
#define CAM_PIN_PCLK    5

const char* ssid = "COSMOTE-536092-2.4GHz";  // Change to your WiFi Network name
const char* password = "vlhelen2003";  // Change to your password

const int LCD_I2C_ADRESS =  0x27;
unsigned long now = millis();
unsigned long previousLcdPrintTime = 0;
LiquidCrystal_I2C Lcd(LCD_I2C_ADRESS, 16, 2); // Create LCD object

DFRobot_AXP313A axp;

camera_fb_t *fb = NULL; // Frame buffer pointer
WebServer server(80);

const int imageWidth = 96;
const int imageHeight = 96;
const int contrastThreshold = 20; // Your required difference threshold
const int bufferLength = imageWidth * imageHeight;


const float Kp_gain = 0.5f; // Kp for proportional correction
const float Target_Row_Fraction = 0.33f; // Look at bottom 33% of image
const float Max_Angle_Degrees = 45.0f; // Max steering limit

// Buffer to store the resulting binary image 
uint8_t current_raw_image_copy[imageWidth * imageHeight];
uint8_t current_binary_image_copy[imageWidth * imageHeight];
uint8_t binary_image_buffer[imageWidth * imageHeight];

volatile float current_target_angle = 0.0f;

void processImage(uint8_t* src_buf, uint8_t* dest_buf, int width, int height) {
    long sum = 0;
    int totalPixels = width * height;

    // Calculate the average brightness of the entire image
    for (int i = 0; i < totalPixels; i++) {
        sum += src_buf[i];
    }

    uint8_t globalThreshold = sum / totalPixels; 

    // Binarize the image based on the global threshold
    for (int i = 0; i < totalPixels; i++) {
        if (src_buf[i] < globalThreshold) {
            // Darker than average 
            dest_buf[i] = 0;   // Black (0)
        } else {
            // Brighter than average 
            dest_buf[i] = 255; // White (1)
        }
    }
}

float calculateTargetAngle(const uint8_t* binary_img, 
                                 int width, 
                                 int height,
                                 float Kp,          
                                 float target_row_fraction,
                                 float max_angle)
{   
    const int start_row = height - static_cast<int>(height * target_row_fraction);
    float total_deviation = 0.0f;
    int valid_rows_found = 0;
    const float center_x = width / 2.0f;

    for (int row = start_row; row < height; ++row) {
        int row_line_pixels = 0;
        int row_column_sum = 0;

        // Find the center of the line for this specific row
        for (int col = 0; col < width; ++col) {
            // Check for black lines on white floor
            if (binary_img[row * width + col] == 0) { 
                row_line_pixels++;
                row_column_sum += col;
            }
        }

        // Only consider this row if a sufficient amount of line was found (> 3 pixels prevents noise)
        if (row_line_pixels > 3) { 
            float row_centroid = (float)row_column_sum / row_line_pixels;
            total_deviation += (row_centroid - center_x);
            valid_rows_found++;
        }
    }

    if (valid_rows_found == 0) {
        return 0.0f; // No reliable line found in any row in the ROI
    }
    
    // Calculate the average deviation across all valid rows
    float average_deviation = total_deviation / valid_rows_found;

    // Convert average deviation to an initial raw angle value (before tuning)
    const float max_deviation = width / 2.0f;
    float raw_angle_output = (average_deviation / max_deviation) * max_angle;

    float target_angle = raw_angle_output * Kp * -1.0f; 

    // Constrain target angle to the steering limit
    if (target_angle > max_angle) target_angle = max_angle;
    if (target_angle < -max_angle) target_angle = -max_angle;

    return target_angle;
}


// HTML and Javascript for the webpage
void handle_root() {
  String htmlResponse = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>ESP32 Grayscale + Binary</title>
<style> canvas { margin: 10px; } </style>
</head>
<body>
<h1>OV2640 Image Processing</h1>
<p><strong>Target Angle: <span id="angleValue">0.00</span> degrees</strong></p>
<canvas id="cameraCanvasOriginal" width=")rawliteral" + String(imageWidth) + R"rawliteral(" height=")rawliteral" + String(imageHeight) + R"rawliteral(" style="border:1px solid #000;"></canvas>
<canvas id="cameraCanvasBinary" width=")rawliteral" + String(imageWidth) + R"rawliteral(" height=")rawliteral" + String(imageHeight) + R"rawliteral(" style="border:1px solid #000;"></canvas>

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
  // Fetch Original Image
  var xhr1 = new XMLHttpRequest();
  xhr1.open('GET', '/capture', true);
  xhr1.responseType = 'arraybuffer';
  xhr1.onload = function(e) {
    if (xhr1.response) drawToCanvas('cameraCanvasOriginal', new Uint8Array(xhr1.response));
  };
  xhr1.send();

  // Fetch Binary Image
  var xhr2 = new XMLHttpRequest();
  xhr2.open('GET', '/capture_binary', true);
  xhr2.responseType = 'arraybuffer';
  xhr2.onload = function(e) {
    if (xhr2.response) drawToCanvas('cameraCanvasBinary', new Uint8Array(xhr2.response));
  };
  xhr2.send();

  // *** NEW: Fetch Angle ***
  var xhr3 = new XMLHttpRequest();
  xhr3.open('GET', '/angle', true);
  xhr3.onload = function(e) {
    if (xhr3.responseText) {
      document.getElementById('angleValue').innerText = xhr3.responseText;
    }
  };
  xhr3.send();
}

// Update all data simultaneously every 300ms
setInterval(fetchImagesAndAngle, 300); 
fetchImagesAndAngle(); // Call immediately on load
</script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", htmlResponse);
}

void handle_capture() {
  server.send_P(200, "application/octet-stream", (const char*)current_raw_image_copy, (size_t)bufferLength);
}
void handle_capture_binary() {
    server.send_P(200, "application/octet-stream", (const char*)current_binary_image_copy, (size_t)bufferLength);
}
void handle_angle() {
  String angleStr = String(current_target_angle, 2); 
  server.send(200, "text/plain", angleStr);
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  // AXP313A init must happen first before potentially initializing other I2C devices.
  // Initialize power management for the camera 
  while(axp.begin() != 0){
    Serial.println("init error");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

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

  Wire.begin(); //begin i2c AFTER setting camera parameters
  Lcd.init();
  Lcd.backlight();
  Lcd.clear();

  // Initialize the camera (AFTER THE OTHER I2C SENSORS!!!)
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    Serial.println("Check AXP313A power settings and wiring.");
    return;
  }

  Serial.println("Camera initialized successfully for car vision system.");

  // Apply settings
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
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("Web server IP address: http://");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, handle_root);
  server.on("/capture", HTTP_GET, handle_capture);
  server.on("/capture_binary", HTTP_GET, handle_capture_binary);
  server.on("/angle", HTTP_GET, handle_angle); 
  server.begin(); 

  ArduinoOTA.begin();

}

void loop() {
  ArduinoOTA.handle();  
  server.handleClient(); 

  float angle_sum = 0.0f;
  int frames_processed = 0;

  for (int i = 0; i < 2; i++) { // Capture and analyze 2 frames
      camera_fb_t * fb = esp_camera_fb_get();
      if (fb) {
          memcpy(current_raw_image_copy, fb->buf, bufferLength);
          processImage(fb->buf, current_binary_image_copy, imageWidth, imageHeight);
          
          // Pass the necessary parameters as defined by the function signature
          float current_frame_angle = calculateTargetAngle(
              current_binary_image_copy, 
              imageWidth, 
              imageHeight,
              Kp_gain,             
              Target_Row_Fraction, 
              Max_Angle_Degrees    
          );
          
          if (current_frame_angle != 0.0f) { 
            angle_sum += current_frame_angle;
            frames_processed++;
          }
          esp_camera_fb_return(fb); 
      }
  }
  
  if (frames_processed > 0) {
    current_target_angle = angle_sum / frames_processed;
  } else {
    current_target_angle = 0.0f;
  }

  now = millis();
  if (now - previousLcdPrintTime > 100) {
    // Format numbers to a fixed width (e.g., 8 characters with 2 decimal places)
    char targetAngleStr[12]; // Need space for 8 chars + null terminator

    sprintf(targetAngleStr, "%5.2f", current_target_angle); // Right-aligned, 5 chars total, 2 decimal places

    Lcd.setCursor(0, 0);
    Lcd.print(targetAngleStr);

    previousLcdPrintTime = now;
  }

}

