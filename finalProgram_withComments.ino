/********** Firebeetle 2 - ESP32-S3 Autonomous Vehicle System **********
* Purpose
* Implements a smart vehicle platform using an ESP32-S3 microcontroller.
* The system integrates live camera streaming, GPS tracking, WebSocket
* telemetry, motor control with PID-based heading correction, IR remote
* control, LCD feedback, LED signaling, Edge Impulse AI image recognition model 
* integration with web server labeling and multiple safety indicators.
*
* Hardware
* - ESP32-S3 with OV2640 camera (powered by AXP313A PMIC)
* - MPU6050 gyro (I2C @0x68) for heading estimation
* - Neo-6M Gps via UART0 (pins 43/44)
* - MCP23017 I/O expander (I2C @0x20) for DIR1, orange LEDs, buzzer, smoke
* - LCD (I2C @0x27), IR receiver (D2), motor drivers (D3/D6 PWM, D7 DIR2)
* - vibration (A4), ultrasonic (D11/D12)
* Software
* - Uses Arduino core libraries/ESP-IDF 
* - Camera driver: esp_camera
* - Networking: WiFiMulti, WebServer, WebSockets
* - Data encoding: ArduinoJson
* - AI inference: a0912_inferencing
* - GPS parsing: TinyGPS++
*
* Reference
*  * Version: v1.0, Th. Tsantilas, P. Savvidi
* Date: Jan 2026
**********/
#include "DFRobot_AXP313A.h"
#include "esp_camera.h"
#include <WebServer.h>        // Standard WebServer library
#include <esp32-hal-i2c.h>    // Low-level I2C to override the camera library
#include <WiFi.h>
#include <WiFiMulti.h>
#include <ESPmDNS.h>
#include <a0912_inferencing.h> //AI model
#include "edge-impulse-sdk/tensorflow/lite/c/common.h" //TF lite C API
#include <TinyGPS++.h>          
#include <Wire.h>               // I2C 
#include <Adafruit_MCP23X17.h>  // MCP23
#include <LiquidCrystal_I2C.h>  // LCD 
#include <IRremote.h>           // IR 

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

#define GPS_RX_PIN 44 // UART0
#define GPS_TX_PIN 43
#define GPS_BAUD 9600

#define MAX_ULTRASONIC_DISTANCE 200 // Maximum distance in cm

#define HOSTNAME "esp32-data" // DNS localhost address

camera_fb_t* fb = NULL; // Camera frame buffer pointer

const int imageWidth = 96; //Image buffer dimensions
const int imageHeight = 96;
const int bufferLength = imageWidth * imageHeight;
uint8_t currentRawImageCopy[imageWidth * imageHeight];
SemaphoreHandle_t FrameMutex; // Handle to manage shared resources between AI and web server

int previousLabelX = -1; // AI labels
int previousLabelY = -1;
String previousLabel = "";

//IR Receiver
const byte IR_RECEIVE_PIN = D2;
const int DEBOUNCE_DELAY = 250;
//Motor Driver
const byte DIR1_PIN =  15, DIR2_PIN = D7, PWM1_PIN = D3, PWM2_PIN = D6;
//Leds
const byte RED_LED_PIN = 16, WHITE_LED_PIN = 17, ORANGE_LED_PIN1 = 7, ORANGE_LED_PIN2 = 6; //orange mcp
const unsigned long BLINK_PERIOD = 660;
//LCD 
const int LCD_I2C_ADRESS =  0x27;
//Gyro
const int MPU_I2C_ADRESS = 0x68;
const byte GYRO_SAMPLE_RATE = 10; // ms between samples
const byte GYRO_SENSITIVITY = 131; // LSB/deg/s for ±250dps range
const byte GYRO_XOUT_H = 0x43; // Starting register for gyro data
//Vibraton 
const int VIBRATION_PIN = A4;
//Smoke
const int SMOKE_PIN = 4; //mcp  
//Buzzer
const int BUZZER_PIN = 5; //mcp
//Fire Sensor
const byte FLAME_SENSOR_PIN = 3; //mcp
//Ultrasonic
const byte TRIGGER_PIN = D11;
const byte ECHO_PIN = D12;
const float OBSTACLE_THRESHOLD_CM = 5.0f; 
/* IR line sensor pins */
const byte IR_Right = 2; //mcp
const byte IR_Left = 1; //mcp

unsigned long now = millis();
unsigned long previousLcdPrintTime = 0;
int16_t gyroX, gyroY, gyroZ;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long previousTimeGyro = 0;
long gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float startingAbsoluteAngleZ = 0.0;
bool shouldBeMoving = false; 
unsigned long pidLinePreviousTime = 0;

const byte ACCEL_ZOUT_H = 0x3F;
float accelZ;
const float ACCEL_SENSITIVITY = 16384.0;

const char* mdnsName = "esp32s3-data";
bool isStreaming = false;

// Simulated variables (for testing via Serial commands)
double latitude = 37.9838;
double longitude = 23.7275;
bool smoke = false;
bool flame = false;
bool vibration = false;
bool arrival = false;
bool departure = false;
bool busDelay = false;
bool speedLimit = false;

byte currentSpeed = 225;
byte rightMotorSpeed = 0;
byte leftMotorSpeed = 0;
float targetAngle = 0.0;

// Blinking LED state tracking
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
float ultrasonicDistance = 0.0;
bool obstacleDetected = false;
bool wasInAutoMode = false;

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

WebServer Server(80);
DFRobot_AXP313A Axp; // Camera power chip
WiFiMulti WifiMulti;
TinyGPSPlus Gps;
HardwareSerial GpsSerial(0); 
Adafruit_MCP23X17 Mcp; // Port expander
LiquidCrystal_I2C Lcd(LCD_I2C_ADRESS, 16, 2); 

/******** function resetPID
* Purpose
* Resets PID controller state to prevent windup during mode transitions.
* Arguments
* None.
* Results
* - Clears integral term
* - Sets pidPreviousInput = current input
* - Marks next calculation as first run
* - Updates pidPreviousTime to current millis()
* Hardware
* None.
* Software
* Modifies global PID state variables.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void resetPID() {
  pidIntegral = 0.0;
  pidPreviousInput = pidInput;
  pidFirstRun = true;
  pidPreviousTime = millis();
}

/******** function pidCorrection
* Purpose
* Computes PID output for differential steering based on heading error.
* Arguments
* targetAngle: Desired heading relative to start (float, degrees)
* currentAngle: Absolute gyro Z angle (float, degrees)
* baseSpeed: Base motor speed (byte, 0-255)
* Results
* - Calculates PID output with anti-windup
* - Sets left/right motor speeds via analogWrite()
* - Constrains output to [-255, 255]
* Hardware
* Motor drivers on PWM1_PIN, PWM2_PIN.
* Software
* Uses global PID parameters and state.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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
  rightMotorSpeed = constrain(baseSpeed + pidOutput, 0, 255);
  leftMotorSpeed  = constrain(baseSpeed - pidOutput, 0, 255);

  analogWrite(PWM2_PIN, rightMotorSpeed);
  analogWrite(PWM1_PIN, leftMotorSpeed);
}

/******** function gyroBegin
* Purpose
* Initializes MPU6050 gyro by waking it from sleep mode.
* Arguments
* None.
* Results
* - Writes 0x00 to MPU6050 register 0x6B (PWR_MGMT_1)
* Hardware
* MPU6050 on I2C @0x68.
* Software
* Uses Wire library for I2C transaction.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void gyroBegin() {
  Wire.beginTransmission(MPU_I2C_ADRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

/******** function readGyro
* Purpose
* Reads and processes gyro data at fixed intervals, updating angleZ.
* Arguments
* None.
* Results
* - Reads raw gyro X/Y/Z values
* - Applies bias correction and sensitivity scaling
* - Integrates to update angleZ (yaw) using trapezoidal rule
* Hardware
* MPU6050 on I2C @0x68.
* Software
* Uses global gyroBias* and GYRO_* constants.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

/******** function calibrateGyro
* Purpose
* Computes average gyro bias by sampling 100 readings at rest.
* Arguments
* None.
* Results
* - Updates global gyroBiasX/Y/Z with average offset
* - Delays GYRO_SAMPLE_RATE ms between samples
* Hardware
* MPU6050 must be stationary during calibration.
* Software
* Uses direct I2C reads without TinyWire.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

/******** function readAccelZ
* Purpose
* Communicates with the MPU sensor via I2C to retrieve vertical acceleration.
* Arguments
* None.
* Results
* - Requests 2 bytes of data from the ACCEL_ZOUT registers.
* - Combines High and Low bytes into a signed 16-bit integer.
* - Normalizes the raw value by ACCEL_SENSITIVITY.
* - Updates the global "accelZ" variable (measured in g-force).
* Hardware
* MPU-6050/9250 (I2C), I2C Bus (SDA/SCL).
* Software
* Wire.h (I2C Library).
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void readAccelZ() {
  Wire.beginTransmission(MPU_I2C_ADRESS);
  Wire.write(ACCEL_ZOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_I2C_ADRESS, 2, true); 

  int16_t rawZ = Wire.read() << 8 | Wire.read(); 
  accelZ = rawZ / ACCEL_SENSITIVITY; // Result in g's
}

/******** function isPickedUp
* Purpose
* Detects physical handling or displacement of the car based on vertical acceleration.
* Arguments
* None.
* Results
* - Invokes "readAccelZ" to get current g-force data.
* - Compares reading against a resting state (~1.0g).
* - Returns true if acceleration deviates significantly (indicates movement/lifting).
* - Returns false if the device remains relatively stationary.
* Hardware
* MPU-6050/9250 Accelerometer.
* Software
* Threshold-based motion detection logic.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
bool isPickedUp() {
  readAccelZ();
  // Normal stationary Z is ~1.0g
  if (accelZ > 1.5 || accelZ < 0.5) { 
    return true; 
  }
  return false;
}

/******** function forward
* Purpose
* Drives both motors forward at specified speed and sets LED indicators.
* Arguments
* speed: Motor speed (byte, 0-255)
* Results
* - Sets DIR1_PIN (MCP) and DIR2_PIN (GPIO) LOW
* - Sets PWM1/PWM2 to speed
* - Turns off all blinking LEDs (sets solid OFF)
* Hardware
* Motor driver H-bridge control pins.
* Software
* Uses Mcp.digitalWrite() and digitalWrite().
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

  shouldBeMoving = true; 
  //shouldBuzzerBuzz  = false;
  //Mcp.digitalWrite(BUZZER_PIN, LOW);
}

/******** function backward
* Purpose
* Drives both motors backward and activates white LED.
* Arguments
* speed: Motor speed (byte, 0-255)
* Results
* - Sets DIR1_PIN and DIR2_PIN HIGH
* - Sets PWM1/PWM2 to speed
* - Turns on white LED (solid ON, not blinking)
* Hardware
* Motor driver and white LED.
* Software
* None.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

  shouldBeMoving = true; 
  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

/******** function turnLeft
* Purpose
* Turns vehicle left by reversing left motor and activating left orange LED.
* Arguments
* speed: Motor speed (byte, 0-255)
* Results
* - Sets DIR1_PIN HIGH, DIR2_PIN LOW
* - Sets PWM1/PWM2 to speed
* - Blinks left orange LED
* Hardware
* Motor driver and MCP-controlled orange LED.
* Software
* None.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

  shouldBeMoving = true; 
  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
} 

/******** function turnRight
* Purpose
* Turns vehicle right by reversing right motor and activating right orange LED.
* Arguments
* speed: Motor speed (byte, 0-255)
* Results
* - Sets DIR1_PIN LOW, DIR2_PIN HIGH
* - Sets PWM1/PWM2 to speed
* - Blinks right orange LED
* Hardware
* Motor driver and MCP-controlled orange LED.
* Software
* None.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

  shouldBeMoving = true; 
  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

/******** function stop
* Purpose
* Stops motors and blinks red LED to indicate halt.
* Arguments
* None.
* Results
* - Sets PWM1/PWM2 to 0
* - Enables red LED blinking
* - Turns off other LEDs
* Hardware
* Motors and red LED.
* Software
* None.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

  shouldBeMoving = false; 
  /*shouldBuzzerBuzz = true;
  buzzerLastTime = now;
  buzzerState = HIGH;
  Mcp.digitalWrite (BUZZER_PIN, buzzerState);*/
}

/******** function manual_mode
* Purpose
* Switches vehicle to manual IR control mode.
* Arguments
* None.
* Results
* - Sets global manualMode = true
* - Calls stop()
* - Prints "MANUAL MODE" to Serial
* Hardware
* None.
* Software
* None.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void manual_mode() {
  manualMode = true;
  stop(); 
  Serial.println("MANUAL MODE");
}

/******** function automaticMode
* Purpose
* Switches vehicle to automatic PID steering mode.
* Arguments
* None.
* Results
* - Sets manualMode = false
* - Stops motors and disables all LED blinking
* - Sets startingAbsoluteAngleZ = current angleZ
* - Resets targetAngle and PID state
* - Prints "AUTOMATIC MODE" to Serial
* Hardware
* None.
* Software
* Calls resetPID().
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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

/******** function checkRedLeds
* Purpose
* Toggles red LED state if blinking is enabled and period elapsed.
* Arguments
* None.
* Results
* - Updates RED_LED_PIN state based on BLINK_PERIOD
* - Updates previousTimeRED and previousRedLedState
* Hardware
* Red LED on GPIO 16.
* Software
* Uses global now and blink flags.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void checkRedLeds() {
  if (shouldRedLedsBlink) {
    if (now - previousTimeRED >= BLINK_PERIOD) {
      previousRedLedState = !previousRedLedState;
      digitalWrite(RED_LED_PIN, previousRedLedState);
      previousTimeRED = now;
    }
  } else {
    digitalWrite(RED_LED_PIN, LOW);
    previousRedLedState = LOW; 
  }
}

/******** function checkWhiteLeds
* Purpose
* Toggles white LED state if blinking is enabled and period elapsed.
* Arguments
* None.
* Results
* - Updates WHITE_LED_PIN state based on BLINK_PERIOD
* - Updates previousTimeWHITE and previousWhiteLedState
* Hardware
* White LED on GPIO 17.
* Software
* Uses global now and blink flags.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void checkWhiteLeds() {
  if (now - previousTimeWHITE >= BLINK_PERIOD && shouldWhiteLedsBlink) {
    previousWhiteLedState = !previousWhiteLedState;
    digitalWrite(WHITE_LED_PIN, previousWhiteLedState);
    previousTimeWHITE = now;
  } 
}

/******** function checkLeftOrangeLed
* Purpose
* Toggles left orange LED state if blinking is enabled and period elapsed.
* Arguments
* None.
* Results
* - Updates MCP pin ORANGE_LED_PIN1 state
* - Updates previousTimeLeftORANGE and state
* Hardware
* Orange LED on MCP23017 pin 7.
* Software
* Uses Mcp.digitalWrite().
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void checkLeftOrangeLed() {
  if (now - previousTimeLeftORANGE >= BLINK_PERIOD && shouldLeftOrangeLedBlink) {
    previousLeftOrangeLedState = !previousLeftOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN1, previousLeftOrangeLedState);
    previousTimeLeftORANGE = now;
  } 
}

/******** function checkRightOrangeLed
* Purpose
* Toggles right orange LED state if blinking is enabled and period elapsed.
* Arguments
* None.
* Results
* - Updates MCP pin ORANGE_LED_PIN2 state
* - Updates previousTimeRightORANGE and state
* Hardware
* Orange LED on MCP23017 pin 6.
* Software
* Uses Mcp.digitalWrite().
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void checkRightOrangeLed() {
  if (now - previousTimeRightORANGE >= BLINK_PERIOD && shouldRightOrangeLedBlink) {
    previousRightOrangeLedState = !previousRightOrangeLedState;
    Mcp.digitalWrite(ORANGE_LED_PIN2, previousRightOrangeLedState);
    previousTimeRightORANGE = now;
  }
}
  
void handleObstacle() {
  if (ultrasonicDistance <= OBSTACLE_THRESHOLD_CM && !manualMode) {
    obstacleDetected = true;
  } else {
    obstacleDetected = false;
  }
}


/******** function handleIRcommunication
* Purpose
* Decodes IR commands and executes vehicle actions or mode changes.
* Arguments
* None.
* Results
* - Ignores repeats and debounces rapid presses
* - Executes movement, mode switch, or target angle change based on command
* - Updates currentSpeed on volume buttons
* Hardware
* IR receiver on D2.
* Software
* Uses IRremote library. Command mapping defined in switch-case.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
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
        shouldBeMoving = true; 
        }
        break; 
      case 82:  backward(currentSpeed); shouldBeMoving = true; break;
      case 8:   if(manualMode) turnLeft(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = 90;  
        shouldBeMoving = true; 
        }
        break;
      case 90:  if(manualMode) turnRight(currentSpeed); else {
        startingAbsoluteAngleZ = angleZ; 
        targetAngle = -90; 
        shouldBeMoving = true; 
        }
        break;
      case 28:  stop();                   break;
      case 69:  manual_mode();            break;
      case 71:  automaticMode();          break;

      // Half turns
      case 12:  targetAngle = 45;      shouldBeMoving = true;      break;
      case 94:  targetAngle = -45;     shouldBeMoving = true;     break;
      case 66:  targetAngle = -45;     shouldBeMoving = true;     break;
      case 74:  targetAngle = 45;      shouldBeMoving = true;     break;

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

/******** function ultrasonicTask
* Purpose
* Background task for distance measurement using an HC-SR04 ultrasonic sensor.
* Arguments
* - pvParameters: Standard FreeRTOS task parameters (not used).
* Results
* - Triggers the ultrasonic sensor with a 10us pulse.
* - Measures echo duration with a 20ms timeout.
* - Calculates distance in centimeters. 
* - Updates the global variable "ultrasonicDistance".
* Hardware
* HC-SR04 Ultrasonic Sensor (Trigger/Echo pins).
* Software
* FreeRTOS, Arduino `pulseIn` function.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void ultrasonicTask(void* pvParameters) {
  while (1) { 
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 20000); 

    if (duration == 0) {
       
    } else {
      ultrasonicDistance = duration * 0.034 / 2;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

/******** function handleServerRoot
* Purpose
* Serves the primary webpage, including HTML and JavaScript.
* Arguments
* None.
* Results
* - Generates a dynamic HTML page with a Canvas sized to imageWidth/imageHeight.
* - Includes JavaScript to fetch binary grayscale images from "/capture".
* - Includes JavaScript to fetch and overlay AI detection data from "/data".
* - Renders detection points and labels directly onto the camera stream.
* - Implements an auto-refresh loop for real-time monitoring.
* Hardware
* Client-side browser (Rendering), ESP32 (Serving).
* Software
* HTML5 Canvas, JavaScript.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void handleServerRoot() {
  String HtmlResponse = R"rawliteral(
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

function fetchImageAndLabels() {
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
setInterval(fetchImageAndLabels, 300); 
fetchImageAndLabels(); // Call immediately on load
</script>
</body>
</html>
)rawliteral";
  Server.send(200, "text/html", HtmlResponse);
}

/******** function handleServerImageCapture
* Purpose
* Serves the raw image buffer to the client as a binary data stream.
* Arguments
* None.
* Results
* - Streams the `currentRawImageCopy` buffer as "application/octet-stream".
* - Allows remote clients/web interfaces to retrieve the exact frame used for AI inference.
* Hardware
* WiFi/Network interface, Camera Buffer memory.
* Software
* ESP32 WebServer (send_P for PROGMEM/Flash-compatible streaming).
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void handleServerImageCapture() {
  Server.send_P(200, "application/octet-stream", (const char*)currentRawImageCopy, (size_t)bufferLength);
}

/******** function handleServerData
* Purpose
* HTTP handler that serves the latest AI detection results in JSON format.
* Arguments
* None.
* Results
* - Constructs a JSON string containing object coordinates and labels.
* - Sends a 200 OK HTTP response with "application/json" content type.
* - Provides data for remote monitoring or web dashboards.
* Hardware
* WiFi/Network interface.
* Software
* ESP32 WebServer, JSON serialization.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void handleServerData() {
  String Json = "{";
  Json += "\"x\":" + String(previousLabelX) + ",";
  Json += "\"y\":" + String(previousLabelY) + ",";
  Json += "\"label\":\"" + previousLabel + "\"";
  Json += "}";
  Server.send(200, "application/json", Json);
}

/******** function convertBufferForAI
* Purpose
* Data transformation callback: normalizes raw pixel data for the AI model.
* Arguments
* - offset: The starting position in the source image array.
* - length: The number of pixels to process in this batch.
* - targetBuffer: Pointer to the float array where processed data is stored.
* Results
* - Offsets pixel values by -128 (Zero-centering/Normalization).
* - Casts 8-bit image data to float format required by Edge Impulse DSP.
* - Returns 0 upon successful completion.
* Hardware
* CPU (Memory operations).
* Software
* Edge Impulse Signal Processing (DSP) pipeline.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
static int convertBufferForAI(size_t offset, size_t length, float* targetBuffer) {
    for (size_t i = 0; i < length; i++) {
        targetBuffer[i] = (float)((int16_t)currentRawImageCopy[offset + i] - 128);
    }
    return 0;
}

/******** function inferenceTask
* Purpose
* Background AI processing task that performs object detection on the latest frame.
* Arguments
* - pvParameters: Pointer to task parameters (required by FreeRTOS, unused).
* Results
* - Synchronizes frame access via "FrameMutex".
* - Executes Edge Impulse "run_classifier" to detect objects.
* - Filters results based on a 0.2 confidence threshold.
* - Updates global variables: previousLabel, previousLabelX, and previousLabelY.
* - Defaults to "Searching..." if no objects are found.
* Hardware
* Camera sensor and CPU/AI Accelerator.
* Software
* FreeRTOS, Edge Impulse SDK (FOMO/Object Detection), TinyML.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void inferenceTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(FrameMutex, pdMS_TO_TICKS(500))) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // Expected: 96*96 = 9216
        signal.get_data = &convertBufferForAI; // Applies pixel - 128
        
        ei_impulse_result_t result = {0}; // Initialize result struct

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK) {
          //Serial.printf("Inference done (%d ms).\n", result.timing.classification);
          
          bool foundObject = false;
          // Iterate through all bounding boxes 
          for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
            auto bb = result.bounding_boxes[ix];
            // Check if the bounding box is valid and meets the confidence threshold
            if (bb.value > 0.2f) { 
              //Serial.printf("  DETECTED: %s (Conf: %.3f) [ x: %u, y: %u, w: %u, h: %u ]\n", 
              //                    bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
              previousLabelX = bb.x;
              previousLabelY = bb.y;
              previousLabel = String(bb.label);

              
              foundObject = true;
              // Break after the first detection above the threshold (or remove break to show all)
              break; 
            }
          }
          if (!foundObject) {
            //erial.println("  No objects detected above threshold.");  
            previousLabelX = -1; 
            previousLabelY = -1;
            previousLabel = "Searching..."; 
          }
        } else {
            // Log errors from the classifier
            //Serial.printf("run_classifier failed with error: %d\n", res);
            previousLabelX = -1; 
            previousLabelY = -1;
            previousLabel = "Error"; 
        }
        
        xSemaphoreGive(FrameMutex);
    } else {
        Serial.println("Inference task: Semaphore timeout.");
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }

}

/******** function setup
* Purpose
* Initializes all hardware peripherals, connects to WiFi, and starts services.
* Arguments
* None.
* Results
* - Configures Serial, GPS, I2C, LCD, MCP, IR, motors, LEDs, ultrasonic
* - Connects to WiFi using fallback networks
* - Starts mDNS and initializes AXP313A for camera power
* - Configures and initializes OV2640 camera
* - Sets up web server routes and WebSocket
* Hardware
* All connected sensors and actuators.
* Software
* Uses all included libraries. Critical: camera init AFTER other I2C devices.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void setup() {
  Serial.begin(115200);
  delay(1000);
  
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

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Mcp.pinMode(IR_Right, INPUT);
  Mcp.pinMode(IR_Left, INPUT);
  Mcp.pinMode(FLAME_SENSOR_PIN, INPUT);
  Mcp.pinMode(SMOKE_PIN, INPUT);
  Mcp.pinMode(BUZZER_PIN, OUTPUT);

  while(Axp.begin() != 0){ 
    delay(500); 
    Serial.println("Camera power init failed!"); 
  }
  Axp.enableCameraPower(Axp.eOV2640);

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
  s->set_aec_value(s, 1200);   // 0 to 1200
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
  WifiMulti.addAP("realme C33", "ibanezfender"); 
  
  while (WifiMulti.run() != WL_CONNECTED) { 
    delay(500); 
    Serial.println("Wifi init failed!"); 
  }
  MDNS.begin(HOSTNAME);
  Server.on("/", HTTP_GET, handleServerRoot);
  Server.on("/capture", HTTP_GET, handleServerImageCapture);
  Server.on("/data", handleServerData);
  Server.begin();

  FrameMutex = xSemaphoreCreateMutex(); // Creates the mutex, mutexes lock memory so shared resources are not overwritten by multiple tasks/threads/cores at the same time
  
  xTaskCreatePinnedToCore(inferenceTask, "Inference", 16384, NULL, 1, NULL, 1); //task function name, name for debugging, stack size in bytes, inputs, priority, handle

  xTaskCreate(ultrasonicTask, "Ultrasonic", 2048, NULL, 1, NULL);

  Lcd.setCursor(0, 0);
  Lcd.print("Bus Departed");
  departure = true;

  //Mcp.digitalWrite(BUZZER_PIN, HIGH);
  //delay(500);
  //Mcp.digitalWrite(BUZZER_PIN, LOW);
}


/******** function loop
* Purpose
* Main control loop: handles networking, sensors, user input, and actuation.
* Arguments
* None.
* Results
* - Services HTTP and WebSocket clients
* - Processes IR commands
* - Handles Serial debug commands (e.g., "smoke", "reset")
* - Broadcasts sensor data every 2 seconds
* - Reinitializes MCP if disconnected
* - Updates LED blink states
* - Feeds GPS data to TinyGPS++
* - Reads gyro and runs PID if in automatic mode
* - Updates LCD with target and current angles
* Hardware
* All peripherals.
* Software
* Orchestrates all subsystems at ~10ms loop rate.
* Reference
* v1.0, Th. Tsantilas, P. Savvidi, Jan. 2026.
**********/
void loop() {
  now = millis();
  Server.handleClient(); 

  camera_fb_t * fb = esp_camera_fb_get();
  if (fb) {
    if (fb->len == bufferLength) { 
      if (xSemaphoreTake(FrameMutex, pdMS_TO_TICKS(50))) {
        memcpy(currentRawImageCopy, fb->buf, fb->len);
        xSemaphoreGive(FrameMutex);
      }
    } else {
      
      Serial.printf("Wrong frame size: %zu\n", fb->len);
    }
    esp_camera_fb_return(fb); 
  }

  handleIRcommunication();

  if (Serial.available() > 0) {
    String Command = Serial.readStringUntil('\n');
    Command.trim(); 
    if (Command == "update") {
      latitude += (random(-100, 100) / 1000.0);
      longitude += (random(-100, 100) / 1000.0);
      Serial.printf("New Coordinates: Lat: %f, Lng: %f\n", latitude, longitude);
    } 
    else if (Command == "smoke") {
      smoke = true;
      Serial.println("Smoke Detected!");
      Lcd.setCursor(0, 1);
      Lcd.print("Smoke!      ");
    } 
    else if (Command == "flame") {
      flame = true;
      Serial.println("Fire Detected!");
      Lcd.setCursor(0, 1);
      Lcd.print("Fire!      ");
    } 
    else if (Command == "vibration") {
      vibration = true;
      Serial.println("Vibrations Detected!");
      Lcd.setCursor(0, 1);
      Lcd.print("Vibrations!");
    } 
    else if (Command == "arrival") {
      arrival = true;
      Serial.println("Arrived!");
    } 
    else if (Command == "departure") {
      departure = true;
      Serial.println("Departure!");
    } 
    else if (Command == "busDelay") {
      busDelay = true; 
      Serial.println("Delays!");
    } 
    else if (Command == "speedLimit") {
      speedLimit = true; 
      Serial.println("Speed Limit!");
      Lcd.setCursor(0, 1);
      Lcd.print("Flying!    ");
    } 
    else if (Command == "reset") {
      smoke = false;
      flame = false;
      vibration = false;
      arrival = false;
      departure = false;
      busDelay = false;
      speedLimit = false;
      Serial.println("Reset!");
    }
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
      Mcp.pinMode(IR_Right, INPUT);
      Mcp.pinMode(IR_Left, INPUT);
    }
  }
  

  checkRedLeds();
  checkWhiteLeds();
  checkLeftOrangeLed();
  checkRightOrangeLed();


  while (GpsSerial.available() > 0) {
    if (Gps.encode(GpsSerial.read())) {
      if (Gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.print(Gps.location.lat(), 6); 
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
  }
  
  readGyro();
  handleObstacle();
  

  if (!manualMode) { 
    bool isBackwards = false;

    if (shouldBeMoving && !obstacleDetected) {
      byte rightLineReading = Mcp.digitalRead(IR_Right);
      byte leftLineReading = Mcp.digitalRead(IR_Left);
      Serial.println((String)rightLineReading + "    " + leftLineReading);
      if (rightLineReading == HIGH && leftLineReading == LOW) {
        targetAngle = 0.0;
      } else if (rightLineReading == HIGH && leftLineReading == HIGH) {
        targetAngle += 10.0;
      } else if (rightLineReading == LOW && leftLineReading == HIGH) {
        targetAngle += 10.0;
      } else if (rightLineReading == LOW && leftLineReading == LOW) {
        targetAngle -= 10.0;
      }  
      
      
      Mcp.digitalWrite(DIR1_PIN, LOW);
      digitalWrite(DIR2_PIN, LOW);
      isBackwards = false;

      float dtLine = (now - pidLinePreviousTime) / 1000.0;
      if (dtLine >= 0.01) {    
        float alpha = 0.2; //takes 20% of the new angle and adds it to the 80% of the previous angle to smoothly turn
        static float filteredTargetAngle = 0;
        
        filteredTargetAngle = (alpha * targetAngle) + ((1.0 - alpha) * filteredTargetAngle);
        pidCorrection(filteredTargetAngle, angleZ, currentSpeed);
        pidLinePreviousTime = now;
      
      
        Serial.println(filteredTargetAngle);
      }
    } else if (obstacleDetected) {
      Mcp.digitalWrite(DIR1_PIN, HIGH);
      digitalWrite(DIR2_PIN, HIGH);
      isBackwards = true;
      analogWrite(PWM1_PIN, 200);
      analogWrite(PWM2_PIN, 200);
      leftMotorSpeed = 200;
      rightMotorSpeed = 200;
    } 
    else {
      stop();   
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
    }
  
    Serial.println((String)leftMotorSpeed + " ||  " + rightMotorSpeed);
    if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
      shouldRedLedsBlink = true;
      shouldWhiteLedsBlink = false;
      shouldLeftOrangeLedBlink = false;
      shouldRightOrangeLedBlink = false;
    } 
    else if (isBackwards) { 
      shouldRedLedsBlink = false;
      shouldWhiteLedsBlink = true;
      shouldLeftOrangeLedBlink = false;
      shouldRightOrangeLedBlink = false;
    }
    else if (leftMotorSpeed > rightMotorSpeed + 80) {
      shouldRedLedsBlink = false;
      shouldWhiteLedsBlink = false;
      shouldLeftOrangeLedBlink = false;
      shouldRightOrangeLedBlink = true;
      Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
    } 
    else if (leftMotorSpeed < rightMotorSpeed - 80) {
      shouldRedLedsBlink = false;
      shouldWhiteLedsBlink = false;
      shouldLeftOrangeLedBlink = true;
      shouldRightOrangeLedBlink = false;
      Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);
    } 
    else {
      //default
      shouldRedLedsBlink = false;
      shouldWhiteLedsBlink = false;
      shouldLeftOrangeLedBlink = false;
      shouldRightOrangeLedBlink = false;
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(WHITE_LED_PIN, LOW);
      Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
      Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);
    }

  }

  

  if (Mcp.digitalRead(FLAME_SENSOR_PIN)) {
    flame = true;
    Lcd.setCursor(0, 1);
    Lcd.print("Fire!      ");
  } else {
    flame = false;
  }

  if (Mcp.digitalRead(SMOKE_PIN)) {
    smoke = true;
    Lcd.setCursor(0, 1);
    Lcd.print("Smoke!      ");
  } else {
    smoke = false;
  }

  if (isPickedUp()) {
    speedLimit = true;
    Lcd.setCursor(0, 1);
    Lcd.print("Flying!    ");
  } else {
    speedLimit = false;
  }
 
  if (analogRead(VIBRATION_PIN) > 500) {
    vibration = true;
    Lcd.setCursor(0, 1);
    Lcd.print("Vibrations!");
  } else {
    vibration = false;
  }

  if (ultrasonicDistance < 10.0f) {
      shouldBeMoving = false;
  } else if (ultrasonicDistance < 500.0f) {
      shouldBeMoving = true;
  }

  //Serial.println((String)"Ultrasonic:" + ultrasonicDistance);

  delay(1);
}
