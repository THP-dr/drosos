#include <Wire.h>               // I2C 
#include <IRremote.h>           // IR 
#include <LiquidCrystal_I2C.h>  // LCD 
#include <WiFi.h>               // Connect ESP32 to WiFi
#include <ArduinoOTA.h>         // Enable over-the-air updates
#include <WiFiMulti.h>          // Connect to multiple WiFi networks
#include <Adafruit_MCP23X17.h>  // MCP23
#include <TinyGPS++.h>          //GPS
#include "DFRobot_AXP313A.h"
#include "esp_camera.h"




//IR Receiver
const byte IR_RECEIVE_PIN = D2;
const int DEBOUNCE_DELAY = 250;
//Motor Driver
const byte DIR1_PIN =  15, DIR2_PIN = D7, PWM1_PIN = D3, PWM2_PIN = D6;
//Leds
const byte RED_LED_PIN = 16, WHITE_LED_PIN = 17, ORANGE_LED_PIN1 = 6, ORANGE_LED_PIN2 = 7; //orange mcp
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
//Ultrasonic
const byte trigPin = D11;
const byte echoPin = D12;
// GY-NEO6MV2 settings
#define GPS_BAUD 9600 
// UART1 mapping for ESP32-S3 FireBeetle 2
#define RXD1 44
#define TXD1 43

#define CAMERA_MODEL_DFRobot_FireBeetle2_ESP32S3
#include "camera_pins.h"


unsigned long now = millis();

DFRobot_AXP313A axp;
Adafruit_MCP23X17 Mcp;

int16_t gyroX, gyroY, gyroZ;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long previousTimeGyro = 0;
long gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float startingAbsoluteAngleZ = 0.0;

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

//Buzzer
bool shouldBuzzerBuzz = false;
unsigned long buzzerLastTime = 0;
bool buzzerState = LOW;
const unsigned long BUZZER_INTERVAL = 500;


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

bool manualMode = true;   // ξεκινάει σε manual mode

unsigned long previousIRCommandTime = 0;
byte previousCommand = 0;

unsigned long previousLcdPrintTime = 0;

float distance;

// Create TinyGPS++ and Serial objects
TinyGPSPlus gps;
HardwareSerial gpsSerial(1); // Use UART1 for GPS


LiquidCrystal_I2C Lcd(LCD_I2C_ADRESS, 16, 2); // Create LCD object
WiFiMulti wifiMulti;                          // Create WiFiMulti object


void startCameraServer();
void setupLedFlash(int pin);

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

  shouldBuzzerBuzz  = false;
  Mcp.digitalWrite(BUZZER_PIN, LOW);
 


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


  shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);



}

void turn_left(byte speed) {

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

  shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);


} 

void turn_right(byte speed) {

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

  shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);

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

  shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);
}

void manual_mode() {
  manualMode = true;
  stop(); 

  //keeping it ?????
  /*shouldRedLedsBlink = false;
  shouldWhiteLedsBlink = false;
  shouldLeftOrangeLedBlink = false;
  shouldRightOrangeLedBlink = false;

  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(WHITE_LED_PIN, LOW);
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);*/

  Serial.println("MANUAL MODE");
}

void automatic_mode() {
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


  Mcp.digitalWrite(BUZZER_PIN, LOW);
  

  startingAbsoluteAngleZ = angleZ;  // Store the current angle as the reference
  targetAngle = 0.0;  // Reset current heading as reference point
  resetPID();  // Clear accumulated errors
  Serial.println("AUTOMATIC MODE");

  shouldBuzzerBuzz = false;
  Mcp.digitalWrite(BUZZER_PIN ,LOW);
}

void check_red_leds() {
  if (now - previousTimeRED >= BLINK_PERIOD && shouldRedLedsBlink) {
    previousRedLedState = !previousRedLedState;
    digitalWrite(RED_LED_PIN, previousRedLedState);
    previousTimeRED = now;
  }
}

void check_white_leds() {
  if (now - previousTimeWHITE >= BLINK_PERIOD && shouldWhiteLedsBlink) {
    previousWhiteLedState = !previousWhiteLedState;
    digitalWrite(WHITE_LED_PIN, previousWhiteLedState);
    previousTimeWHITE = now;
  }
}

void check_left_orange_led() {
  if (now - previousTimeLeftORANGE >= BLINK_PERIOD && shouldLeftOrangeLedBlink) {
    previousLeftOrangeLedState = !previousLeftOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN1, previousLeftOrangeLedState);
    previousTimeLeftORANGE = now;
  }
}

void check_right_orange_led() {
  if (now - previousTimeRightORANGE >= BLINK_PERIOD && shouldRightOrangeLedBlink) {
    previousRightOrangeLedState = !previousRightOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN2, previousRightOrangeLedState);
    previousTimeRightORANGE = now;
  }
}

void check_buzzer() {
  if (shouldBuzzerBuzz && (now - buzzerLastTime >= BUZZER_INTERVAL)) {
  buzzerState = !buzzerState;
  Mcp.digitalWrite (BUZZER_PIN,buzzerState);
  buzzerLastTime = now;
  
  }

}

void gyro_begin() {
  Wire.beginTransmission(MPU_I2C_ADRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void read_gyro() {
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
    delayMicroseconds(100);
  }
}

void calibrate_gyro() {
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

void lcd_begin() {
  Lcd.init();
  Lcd.backlight();
  Lcd.clear();
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

void triggerSensor() {      
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}



float readDistance() {
  triggerSensor();
  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout 
  if (duration == 0) {
    return -1; // No object detected
    Serial.println("No pulse");
  }
  return duration * 0.0343 / 2; // Convert to cm
}


void handle_IR_communication() {
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
        //resetPID();
        }
        break; 
      case 82:  backward(currentSpeed); break;
      case 8:   if(manualMode) turn_left(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = 90;  
        //resetPID();
        }
        break;
      case 90:  if(manualMode) turn_right(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = -90; 
        //resetPID();
        }
        break;
      case 28:  stop();                    break;
      case 69:  manual_mode();             break;
      case 71:  automatic_mode();          break;

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

// Add multiple networks with their passwords
void setupWiFiNetworks() {
  wifiMulti.addAP("COSMOTE-536092-2.4GHz", "vlhelen2003");  // Primary network
  wifiMulti.addAP("PasadesOnly", "AnythingGoes");  // Fallback network
  wifiMulti.addAP("realme C33", "ibanezfender");  // Another fallback
  // Add many networks
  // wifiMulti.addAP("Another Network", "Another Password");
}

void setup() {
  Serial.begin(115200);  
  Serial.setDebugOutput(true);
  delay(1000);           // USB CDC takes a while

    while(axp.begin() != 0){
    Serial.println("init error");
    delay(1000);
  }
  axp.enableCameraPower(axp.eOV2640);

  Wire.begin();

      // mcp.begin
  if (!Mcp.begin_I2C()) {
  //if (!mcp.begin_SPI(CS_PIN)) {
    Serial.println(" Mcp Error.");
  }
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

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD1, TXD1);  // Initialize UART1 with your defined pins
  lcd_begin();
  setupWiFiNetworks();   // Add all networks
  Serial.println("Connecting to WiFi...");
    // Connect to the best available network
  while(wifiMulti.run() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  ArduinoOTA.begin();  // Start OTA updates

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_96X96;
  config.pixel_format = PIXFORMAT_GRAYSCALE; //PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  //config.jpeg_quality = 12;
  config.fb_count = 1;

      // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_96X96;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

      // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Setup LED FLash if LED pin is defined in camera_pins.h
  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif


  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);
  Serial.println("Reset reason: " + String(esp_reset_reason()));
  gyro_begin();
  calibrate_gyro();
  Serial.println("--- GPS Coordinate Feed ---");
}

void loop() {
  ArduinoOTA.handle();  // Continuously check for OTA updates (maybe remap to a switch/button to save time and battery?)
  now = millis();  
  // Check WiFi connection and reconnect if needed
  if(wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    //wifiMulti.run() will automatically try to reconnect to the best available network
  }
  
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      displayInfo();
    }
  }
  // Warning if no data is received at all
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    
  }

  handle_IR_communication();

  check_red_leds();
  check_white_leds();
  check_left_orange_led();
  check_right_orange_led();
  check_buzzer();
  read_gyro();

  
  int vibrationSensorValue = analogRead(VIBRATION_PIN);
  int smokeSensorValue = Mcp.digitalRead(SMOKE_PIN);
  //int smokeSensorValue = analogRead(SMOKE_PIN);
  distance = readDistance();

  Serial.print(vibrationSensorValue );
  Serial.print("     ");  
  Serial.println(smokeSensorValue);
  Serial.print("     ");  

  bool obstacleDetected = (distance >= 0 && distance <= 10.0); 
  if (obstacleDetected) {
    Serial.println("OBSTACLE DETECTED : STOP!");
    stop();
    
 
  } else {
    if (distance < 0) {
      Serial.println("No object detected : GO!");

    
    } else {
      Serial.print("Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
      Serial.println("Path clear : GO!");
     
    }
  }
  
  if (!manualMode){
    pidCorrection(targetAngle, angleZ, currentSpeed);
  }


  if (now - previousLcdPrintTime > 100) {
    delayMicroseconds(500);
    // Format numbers to a fixed width (e.g., 8 characters with 2 decimal places)
    char targetAngleStr[9]; // Need space for 8 chars + null terminator
    char angleZStr[9];

    sprintf(targetAngleStr, "%5.2f", targetAngle); // Right-aligned, 5 chars total, 2 decimal places
    sprintf(angleZStr, "%5.2f", angleZ);           // Right-aligned, 5 chars total, 2 decimal places

    Lcd.setCursor(0, 0);
    Lcd.print(targetAngleStr);
    Lcd.setCursor(0, 1);
    Lcd.print(angleZStr);

    previousLcdPrintTime = now;
  }
  delay (100); //for stability
}

void displayInfo() {
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(" | Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for satellite fix...");
  }
}
