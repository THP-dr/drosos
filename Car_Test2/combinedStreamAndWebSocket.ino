#include "esp_camera.h"
#include "DFRobot_AXP313A.h"  // Include AXP313A library 
#include <esp32-hal-i2c.h>    // Low-level I2C to override the camera library
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

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

#define PART_BOUNDARY "123456789000000000000987654321" //random string that is unique, it must be different from everything else sent to the server
static const char* STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
//these will appear in the server as such
/*[Header] Content-Type: multipart/x-mixed-replace; boundary=12345...
[Boundary] --12345...
[Part] Content-Type: image/jpeg, Content-Length: 12500
[RAW DATA OF IMAGE 1]
[Boundary] --12345...
[Part] Content-Type: image/jpeg, Content-Length: 12480
[RAW DATA OF IMAGE 2]
*/

WiFiMulti WifiMulti;
DFRobot_AXP313A Axp;
WebServer Server(80);
WiFiClient StreamingClient;
WebSocketsServer WebSocket = WebSocketsServer(81);

const char* mdnsName = "esp32s3-data";
bool isStreaming = false;

// Simulated variables 
double latitude = 37.9838;
double longitude = 23.7275;
unsigned long lastUpdate = 0;
bool smoke = false;
bool flame = false;
bool vibration = false;
bool arrival = false;
bool departure = false;
bool busDelay = false;
bool speedLimit = false;

// The handler now only "starts" the stream and hands off the client
void handleStream() {
  StreamingClient = Server.client();
  
  StreamingClient.print("HTTP/1.1 200 OK\r\n");
  StreamingClient.print("Content-Type: ");
  StreamingClient.print(STREAM_CONTENT_TYPE);
  StreamingClient.print("\r\nAccess-Control-Allow-Origin: *\r\n\r\n");
  
  isStreaming = true;
  Serial.println("Stream started");
}

void runStreamService() {
  if (!isStreaming) return;

  if (!StreamingClient.connected()) {
    isStreaming = false;
    Serial.println("Stream stopped");
    return;
  }

  // Capture one frame
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) return;

  // Send the multipart frame
  StreamingClient.print(STREAM_BOUNDARY);
  char part_header[64];
  sprintf(part_header, STREAM_PART, fb->len);
  StreamingClient.print(part_header);
  StreamingClient.write(fb->buf, fb->len);
  
  esp_camera_fb_return(fb);
}

void broadcastData() {
  JsonDocument Doc; 
  Doc["lat"] = latitude;
  Doc["lng"] = longitude;
  Doc["smoke"] = smoke;
  Doc["flame"] = flame;
  Doc["vibration"] = vibration;
  Doc["arrival"] = arrival;
  Doc["departure"] = departure;
  Doc["busDelay"] = busDelay;
  Doc["speedLimit"] = speedLimit;
  Doc["uptime"] = millis() / 1000;

  String JsonString;
  serializeJson(Doc, JsonString);

  WebSocket.broadcastTXT(JsonString);
  Serial.println("Broadcasted GPS: " + JsonString);
}

void setup() {
  Serial.begin(115200);

  WifiMulti.addAP("COSMOTE-536092-2.4GHz", "vlhelen2003");

  Serial.println("Connecting to WiFi...");
  while (WifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  if (MDNS.begin(mdnsName)) {
    Serial.println("mDNS started: " + String(mdnsName) + ".local");
    MDNS.addService("ws", "tcp", 81);
  }

  while(Axp.begin() != 0){
    Serial.println("init error");
    delay(1000);
  }
  Axp.enableCameraPower(Axp.eOV2640);

  // Configure camera parameters
  camera_config_t Config;
  Config.ledc_channel = LEDC_CHANNEL_0;
  Config.ledc_timer = LEDC_TIMER_0;
  Config.pin_d0 = CAM_PIN_Y2;
  Config.pin_d1 = CAM_PIN_Y3;
  Config.pin_d2 = CAM_PIN_Y4;
  Config.pin_d3 = CAM_PIN_Y5;
  Config.pin_d4 = CAM_PIN_Y6;
  Config.pin_d5 = CAM_PIN_Y7;
  Config.pin_d6 = CAM_PIN_Y8;
  Config.pin_d7 = CAM_PIN_Y9;
  Config.pin_xclk = CAM_PIN_XCLK;
  Config.pin_pclk = CAM_PIN_PCLK;
  Config.pin_vsync = CAM_PIN_VSYNC;
  Config.pin_href = CAM_PIN_HREF;
  Config.sccb_i2c_port = 0; //IMPORTANT TO LEAVE I2C FOR OTHER SENSORS
  Config.pin_sscb_sda = CAM_PIN_SIOD; 
  Config.pin_sscb_scl = CAM_PIN_SIOC; 
  Config.pin_pwdn = CAM_PIN_PWDN;
  Config.pin_reset = CAM_PIN_RESET;
  Config.xclk_freq_hz = 20000000; // 20MHz
  Config.pixel_format = PIXFORMAT_JPEG; 
  Config.frame_size = FRAMESIZE_VGA; // 640x480 resolution
  Config.jpeg_quality = 12;
  Config.fb_count = 2; // Number of frame buffers
  Config.fb_location = CAMERA_FB_IN_PSRAM; // Use PSRAM 
  Config.grab_mode = CAMERA_GRAB_WHEN_EMPTY; 

  // Initialize the camera (AFTER THE OTHER I2C SENSORS!!!)
  esp_err_t err = esp_camera_init(&Config);
  if (err != ESP_OK) {
    Serial.printf("Camera initialization failed with error 0x%x\n", err);
    Serial.println("Check AXP313A power settings and wiring.");
    return;
  }

  Serial.println("Camera initialized successfully for car vision system.");

  // Apply settings
  sensor_t* s = esp_camera_sensor_get();
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
  s->set_quality(s, 12);
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable

  Server.on("/stream", HTTP_GET, handleStream);
  Server.begin();

  WebSocket.begin();
}

void loop() {
  Server.handleClient(); // Checks for new connection requests
  runStreamService();    // Sends one frame if someone is connected
  
  WebSocket.loop();
    
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim(); 
    if (command == "update") {
      latitude += (random(-100, 100) / 1000.0);
      longitude += (random(-100, 100) / 1000.0);
      Serial.printf("New Coordinates: Lat: %f, Lng: %f\n", latitude, longitude);
      broadcastData(); 
    } 
    else if (command == "smoke") {
      smoke = true;
      Serial.println("Smoke Detected!");
      broadcastData(); 
    } 
    else if (command == "flame") {
      flame = true;
      Serial.println("Fire Detected!");
      broadcastData();
    } 
    else if (command == "vibration") {
      vibration = true;
      Serial.println("Vibrations Detected!");
      broadcastData();
    } 
    else if (command == "arrival") {
      arrival = true;
      Serial.println("Arrived!");
      broadcastData();
    } 
    else if (command == "departure") {
      departure = true;
      Serial.println("Departure!");
      broadcastData();
    } 
    else if (command == "busDelay") {
      busDelay = true; 
      Serial.println("Delays!");
      broadcastData();
    } 
    else if (command == "speedLimit") {
      speedLimit = true; 
      Serial.println("Speed Limit!");
      broadcastData();
    } 
    else if (command == "reset") {
      smoke = false;
      flame = false;
      vibration = false;
      arrival = false;
      departure = false;
      busDelay = false;
      speedLimit = false;
      Serial.println("Reset!");
      broadcastData();
    }
  }

  //send to web socket every 2 seconds
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    broadcastData();
  }
  
  delay(1); 
}