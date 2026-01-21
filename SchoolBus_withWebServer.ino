#include "esp_camera.h"
#include "DFRobot_AXP313A.h"  // Include AXP313A library 
#include <esp32-hal-i2c.h>    // Low-level I2C to override the camera library
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <TinyGPS++.h>
#include <Wire.h>               // I2C 
#include <Adafruit_MCP23X17.h>  // MCP23
#include <LiquidCrystal_I2C.h>  // LCD 
#include <IRremote.h>           // IR 

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

#define GPS_RX_PIN 44 //UART0
#define GPS_TX_PIN 43
#define GPS_BAUD 9600

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

//IR Receiver
const byte IR_RECEIVE_PIN = D2;
const int DEBOUNCE_DELAY = 250;
//Motor Driver
const byte DIR1_PIN =  15, DIR2_PIN = D7, PWM1_PIN = D3, PWM2_PIN = D6;
//Leds
const byte RED_LED_PIN = 16, WHITE_LED_PIN = 17, ORANGE_LED_PIN1 = 7, ORANGE_LED_PIN2 = 6; //orange mcp
const unsigned long BLINK_PERIOD = 660;
//Potentiometer
const byte POT_PIN = A0;
//LCD 
const int LCD_I2C_ADRESS =  0x27;
//Gyro
const int MPU_I2C_ADRESS = 0x68;
const byte GYRO_SAMPLE_RATE = 10;
const byte GYRO_SENSITIVITY = 131;
const byte GYRO_XOUT_H = 0x43;
//Vibraton 
const int VIBRATION_PIN = A4;
//Smoke
const int SMOKE_PIN = 4; //mcp  
//Buzzer
const int BUZZER_PIN = 5; //mcp
//Fire Sensor
const byte FLAME_SENSOR_PIN = 3; //mcp
//Ultrasonic
const byte trigPin = D11;
const byte echoPin = D12;

unsigned long now = millis();
unsigned long previousLcdPrintTime = 0;
int16_t gyroX, gyroY, gyroZ;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long previousTimeGyro = 0;
long gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float startingAbsoluteAngleZ = 0.0;

const byte ACCEL_ZOUT_H = 0x3F;
float accelZ;
const float ACCEL_SENSITIVITY = 16384.0;

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

byte currentSpeed = 225;
float targetAngle = 0.0;

//Leds
bool shouldRedLedsBlink = false, shouldWhiteLedsBlink = false;
bool shouldLeftOrangeLedBlink = false, shouldRightOrangeLedBlink = false;
unsigned long previousTimeRED = 0, previousTimeWHITE = 0;  
unsigned long previousTimeLeftORANGE = 0, previousTimeRightORANGE = 0;
byte previousRedLedState = 0;
byte previousWhiteLedState = 0;
byte previousLeftOrangeLedState = 0, previousRightOrangeLedState = 0;

bool manualMode = true; 
unsigned long previousIRCommandTime = 0;
byte previousCommand = 0;
unsigned long lastMcpCheck = 0;

// PID parameters
float pidKp = 10.0;      // proportionalTerm gain
float pidKi = 0.5;      // Integral gain  
float pidKd = 0.05;     // Derivative gain

// PID state variables
float pidTargetAngle = 0.0;       // Target value
float pidInput = 0.0;          // Current measured value
float pidOutput = 0.0;         // PID output (-255 to 255)
float pidError = 0.0;          // Current error
float pidIntegral = 0.0;       // Accumulated error
float pidDerivative = 0.0;     // Rate of change of error
float pidPreviousInput = 0.0;      // Previous input value
unsigned long pidPreviousTime = 0; // Last calculation time
bool pidFirstRun = true;       // Flag for first calculation

WiFiMulti WifiMulti;
DFRobot_AXP313A Axp;
WebServer Server(80);
WiFiClient StreamingClient;
WebSocketsServer WebSocket = WebSocketsServer(81);
TinyGPSPlus Gps;
HardwareSerial GpsSerial(0); 
Adafruit_MCP23X17 Mcp;
LiquidCrystal_I2C Lcd(LCD_I2C_ADRESS, 16, 2); // Create LCD object

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

void displayInfo() {
  if (Gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(Gps.location.lat(), 6); // 6 decimal places for accuracy
    Serial.print(" | Longitude: ");
    Serial.println(Gps.location.lng(), 6);
    latitude = Gps.location.lat();
    longitude = Gps.location.lng();
  } else {
    if (Gps.charsProcessed() < 10) {
      Serial.println("GPS Error: No data received. Check wiring/baud rate.");
    } else {
      Serial.print("GPS Status: Searching for Satellites... Satellites in view: ");
      Serial.println(Gps.satellites.value());
    }
  }
}

void resetPID() {
  pidIntegral = 0.0;
  pidPreviousInput = pidInput;
  pidFirstRun = true;
  pidPreviousTime = millis();
}

void pidCorrection(float targetAngle, float currentAngle, byte baseSpeed) {
  
  float relativeAngle = currentAngle - startingAbsoluteAngleZ;

  // Update PID state
  pidTargetAngle = targetAngle;
  pidInput = relativeAngle;
  
  float dt = (now - pidPreviousTime) / 1000.0; // Convert to seconds
  
  if (dt >= 0.001) { // At least 1ms has passed
    pidError = pidTargetAngle - pidInput;
    
    // Calculate integral with anti-windup
    pidIntegral += pidError * dt;
    if (pidIntegral > 100.0) pidIntegral = 100.0;
    if (pidIntegral < -100.0) pidIntegral = -100.0;
    
    // Calculate derivative
    if (!pidFirstRun) {
      pidDerivative = (pidInput - pidPreviousInput) / dt;
    } else {
      pidDerivative = 0.0;
      pidFirstRun = false;
    }
     
    // Calculate PID output
    float proportionalTerm = pidKp * pidError;
    float integralTerm = pidKi * pidIntegral;
    float derivativeTerm = pidKd * pidDerivative;
    
    pidOutput = proportionalTerm + integralTerm - derivativeTerm; // Negative derivative term because of inputting process variable instead of error
    
    // Constrain output to valid range
    if (pidOutput > 255.0) pidOutput = 255.0;
    else if (pidOutput < -255.0) pidOutput = -255.0;
    
    // Store values for next calculation
    pidPreviousInput = pidInput;
    pidPreviousTime = now;
  }

  // Calculate motor speeds with PID output
  byte rightMotor = constrain(baseSpeed + pidOutput, 0, 255);
  byte leftMotor  = constrain(baseSpeed - pidOutput, 0, 255);

  analogWrite(PWM2_PIN, rightMotor);
  analogWrite(PWM1_PIN, leftMotor);
}

void gyroBegin() {
  Wire.beginTransmission(MPU_I2C_ADRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readGyro() {
  if (now - previousTimeGyro >= GYRO_SAMPLE_RATE) {
    Wire.beginTransmission(MPU_I2C_ADRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADRESS, 6, true);

    gyroX = Wire.read() << 8 | Wire.read(); 
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();

    gyroX -= gyroBiasX;
    gyroY -= gyroBiasY;
    gyroZ -= gyroBiasZ;

    gyroX /= GYRO_SENSITIVITY;
    gyroY /= GYRO_SENSITIVITY;
    gyroZ /= GYRO_SENSITIVITY;

    float dt = (now - previousTimeGyro) / 1000.0;
    angleX += gyroX * dt;
    angleY += gyroY * dt;
    angleZ += gyroZ * dt;

    previousTimeGyro = now;
  }
}

void calibrateGyro() {
  long gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
  const int samples = 100;  

  for (int gyroReadings = 0; gyroReadings < samples; gyroReadings++) {
    Wire.beginTransmission(MPU_I2C_ADRESS);
    Wire.write(GYRO_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_I2C_ADRESS, 6, true);

    gyroX = Wire.read() << 8 | Wire.read(); 
    gyroY = Wire.read() << 8 | Wire.read();
    gyroZ = Wire.read() << 8 | Wire.read();

    gyroSumX += gyroX;
    gyroSumY += gyroY;
    gyroSumZ += gyroZ;
    delay(GYRO_SAMPLE_RATE); 
  }

  gyroBiasX = gyroSumX / samples;
  gyroBiasY = gyroSumY / samples;
  gyroBiasZ = gyroSumZ / samples;
}

void readAccelZ() {
  Wire.beginTransmission(MPU_I2C_ADRESS);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADRESS, 2, true); 

  int16_t rawZ = Wire.read() << 8 | Wire.read(); 
  accelZ = rawZ / ACCEL_SENSITIVITY; // Result in g's
}

bool isPickedUp() {
  readAccelZ();
  // Normal stationary Z is ~1.0g
  if (accelZ > 1.5 || accelZ < 0.5) { 
    return true; 
  }
  return false;
}

void forward(byte speed) {

  Mcp.digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM1_PIN, speed);
  analogWrite(PWM2_PIN, speed);
  
  shouldRedLedsBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  shouldWhiteLedsBlink = false;
  digitalWrite(WHITE_LED_PIN, LOW);
  shouldLeftOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  shouldRightOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);

  //shouldBuzzerBuzz  = false;
  //Mcp.digitalWrite(BUZZER_PIN, LOW);
}

void backward(byte speed) {

  Mcp.digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);
  analogWrite(PWM1_PIN, speed);
  analogWrite(PWM2_PIN, speed);

  shouldRedLedsBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  shouldWhiteLedsBlink = true;
  digitalWrite(WHITE_LED_PIN, HIGH);
  shouldLeftOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  shouldRightOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);


  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

void turnLeft(byte speed) {

  Mcp.digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, LOW);
  analogWrite(PWM1_PIN, speed);
  analogWrite(PWM2_PIN, speed);
  
  shouldRedLedsBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  shouldWhiteLedsBlink = false;
  digitalWrite(WHITE_LED_PIN, LOW);
  shouldLeftOrangeLedBlink = true;
  Mcp.digitalWrite(ORANGE_LED_PIN1, HIGH);
  shouldRightOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);

  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
} 

void turnRight(byte speed) {

  Mcp.digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, HIGH);
  analogWrite(PWM1_PIN, speed);
  analogWrite(PWM2_PIN, speed);
  
  shouldRedLedsBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  shouldWhiteLedsBlink = false;
  digitalWrite(WHITE_LED_PIN, LOW);
  shouldLeftOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  shouldRightOrangeLedBlink = true;
  Mcp.digitalWrite(ORANGE_LED_PIN2, HIGH);

  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

void stop() {

  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  
  shouldRedLedsBlink = true;
  digitalWrite(RED_LED_PIN, HIGH);
  shouldWhiteLedsBlink = false;
  digitalWrite(WHITE_LED_PIN, LOW);
  shouldLeftOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  shouldRightOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);

  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

void manual_mode() {
  manualMode = true;
  stop(); 
  Serial.println("MANUAL MODE");
}

void automaticMode() {
  manualMode = false;
  stop();
  shouldRedLedsBlink = false;
  shouldWhiteLedsBlink = false;
  shouldLeftOrangeLedBlink = false;
  shouldRightOrangeLedBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(WHITE_LED_PIN, LOW);
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);

  //Mcp.digitalWrite(BUZZER_PIN, LOW);
  
  startingAbsoluteAngleZ = angleZ;  // Store the current angle as the reference
  targetAngle = 0.0;  // Reset current heading as reference point
  resetPID();  // Clear accumulated errors
  Serial.println("AUTOMATIC MODE");

  /*shouldBuzzerBuzz = false;
  Mcp.digitalWrite(BUZZER_PIN ,LOW);*/
}

void checkRedLeds() {
  if (now - previousTimeRED >= BLINK_PERIOD && shouldRedLedsBlink) {
    previousRedLedState = !previousRedLedState;
    digitalWrite(RED_LED_PIN, previousRedLedState);
    previousTimeRED = now;
  }
}

void checkWhiteLeds() {
  if (now - previousTimeWHITE >= BLINK_PERIOD && shouldWhiteLedsBlink) {
    previousWhiteLedState = !previousWhiteLedState;
    digitalWrite(WHITE_LED_PIN, previousWhiteLedState);
    previousTimeWHITE = now;
  }
}

void checkLeftOrangeLed() {
  if (now - previousTimeLeftORANGE >= BLINK_PERIOD && shouldLeftOrangeLedBlink) {
    previousLeftOrangeLedState = !previousLeftOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN1, previousLeftOrangeLedState);
    previousTimeLeftORANGE = now;
  }
}

void checkRightOrangeLed() {
  if (now - previousTimeRightORANGE >= BLINK_PERIOD && shouldRightOrangeLedBlink) {
    previousRightOrangeLedState = !previousRightOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN2, previousRightOrangeLedState);
    previousTimeRightORANGE = now;
  }
}

void handleIRcommunication() {
  if (IrReceiver.decode()) {
    // Ignore repeated signal
    if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
      IrReceiver.resume();
      return;
    }
    // Debounce and ignore rapid presses
    if (now - previousIRCommandTime < DEBOUNCE_DELAY &&
      IrReceiver.decodedIRData.command == previousCommand) {
      IrReceiver.resume();
      return;
    }

    previousIRCommandTime = now;
    previousCommand = IrReceiver.decodedIRData.command;

    switch (IrReceiver.decodedIRData.command) { 
      case 24:  if(manualMode) forward(currentSpeed);   else {
        Mcp.digitalWrite(DIR1_PIN, HIGH);
        digitalWrite(DIR2_PIN, HIGH);
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = 0.0; 

        }
        break; 
      case 82:  backward(currentSpeed); break;
      case 8:   if(manualMode) turnLeft(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = 90;  

        }
        break;
      case 90:  if(manualMode) turnRight(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = -90; 
        }
        break;
      case 28:  stop();                    break;
      case 69:  manual_mode();             break;
      case 71:  automaticMode();          break;

      // Half turns
      case 12:  targetAngle = 45;          break;
      case 94:  targetAngle = -45;         break;
      case 66:  targetAngle = -45;         break;
      case 74:  targetAngle = 45;          break;

      case 21:
        if(currentSpeed + 10 <= 255) currentSpeed += 10;
        analogWrite(PWM1_PIN, currentSpeed);
        analogWrite(PWM2_PIN, currentSpeed);
        break;

      case 7:
        if(currentSpeed - 10 >= 70) currentSpeed -= 10;
        analogWrite(PWM1_PIN, currentSpeed);
        analogWrite(PWM2_PIN, currentSpeed);
        break;

      default:
        break;
    }
  }
  IrReceiver.resume();
  /* Test Results
  69 70 71
  68 64 67
  7  21  9
  22 25  13
  12 24 94
  8  28 90
  66 82 74  
  */
}

void setup() {
  Serial.begin(115200);

  GpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin();
  gyroBegin();
  calibrateGyro();
  Lcd.init();
  Lcd.backlight();
  Lcd.clear();
  if (!Mcp.begin_I2C()) {
    Serial.println(" Mcp Error.");
  }

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);

  Mcp.pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(WHITE_LED_PIN, OUTPUT);
  Mcp.pinMode(ORANGE_LED_PIN1, OUTPUT);
  Mcp.pinMode(ORANGE_LED_PIN2, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Mcp.pinMode(FLAME_SENSOR_PIN, INPUT);
  Mcp.pinMode(SMOKE_PIN, INPUT);
  Mcp.pinMode(BUZZER_PIN, OUTPUT);

  WifiMulti.addAP("COSMOTE-536092-2.4GHz", "vlhelen2003");
  WifiMulti.addAP("PasadesOnly", "AnythingGoes");  // Fallback network
  WifiMulti.addAP("realme C33", "ibanezfender");  // Another fallback

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
  Config.grab_mode = CAMERA_GRAB_LATEST; 

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

  Lcd.setCursor(0, 0);
  Lcd.print("Bus Departed");
  departure = true;

  //Mcp.digitalWrite(BUZZER_PIN, HIGH);
  //delay(500);
  //Mcp.digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  now = millis();
  Server.handleClient(); // Checks for new connection requests
  runStreamService();    // Sends one frame if someone is connected
  
  WebSocket.loop();
    
  handleIRcommunication();

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
      Lcd.setCursor(0, 1);
      Lcd.print("Smoke!      ");
    } 
    else if (command == "flame") {
      flame = true;
      Serial.println("Fire Detected!");
      broadcastData();
      Lcd.setCursor(0, 1);
      Lcd.print("Fire!      ");
    } 
    else if (command == "vibration") {
      vibration = true;
      Serial.println("Vibrations Detected!");
      broadcastData();
      Lcd.setCursor(0, 1);
      Lcd.print("Vibrations!");
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
      Lcd.setCursor(0, 1);
      Lcd.print("Flying!    ");
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

  if (millis() - lastMcpCheck > 5000) {
    lastMcpCheck = millis();
    
    Wire.beginTransmission(0x20); //MCP address
    if (Wire.endTransmission() != 0) {
      Serial.println("MCP Disconnected! Re-initializing...");
      Mcp.begin_I2C(); 
     
      Mcp.pinMode(DIR1_PIN, OUTPUT);
      Mcp.pinMode(ORANGE_LED_PIN1, OUTPUT);
      Mcp.pinMode(ORANGE_LED_PIN2, OUTPUT);
      Mcp.pinMode(FLAME_SENSOR_PIN, INPUT);
      Mcp.pinMode(SMOKE_PIN, INPUT);
      Mcp.pinMode(BUZZER_PIN, OUTPUT);
    }
  }
  
  checkRedLeds();
  checkWhiteLeds();
  checkLeftOrangeLed();
  checkRightOrangeLed();

  // Feed characters from GPS to TinyGPS++
  while (GpsSerial.available() > 0) {
    if (Gps.encode(GpsSerial.read())) {
      displayInfo();
    }
  }
  
  readGyro();
  if (!manualMode){
    pidCorrection(targetAngle, angleZ, currentSpeed);
  }

  if (Mcp.digitalRead(FLAME_SENSOR_PIN)) {
    flame = true;
    broadcastData();
    Lcd.setCursor(0, 1);
    Lcd.print("Fire!      ");
  } else {
    flame = false;
  }

  if (Mcp.digitalRead(SMOKE_PIN)) {
    smoke = true;
    broadcastData();
    Lcd.setCursor(0, 1);
    Lcd.print("Smoke!      ");
  } else {
    smoke = false;
  }

  if (isPickedUp()) {
    speedLimit = true;
    broadcastData();
    Lcd.setCursor(0, 1);
    Lcd.print("Flying!    ");
  } else {
    speedLimit = false;
  }
 
  if (analogRead(VIBRATION_PIN) > 200) {
    vibration = true;
    broadcastData();
    Lcd.setCursor(0, 1);
    Lcd.print("Vibrations!");
  } else {
    vibration = false;
  }


  /*if (now - previousLcdPrintTime > 100) {
    char targetAngleStr[9]; // Need space for 8 chars + null terminator
    char angleZStr[9];

    sprintf(targetAngleStr, "%5.2f", targetAngle); 
    sprintf(angleZStr, "%5.2f", angleZ);           

    Lcd.setCursor(0, 0);
    Lcd.print(targetAngleStr);
    Lcd.setCursor(0, 1);
    Lcd.print(angleZStr);

    previousLcdPrintTime = now;
  }*/
  //delay(1); 
}
