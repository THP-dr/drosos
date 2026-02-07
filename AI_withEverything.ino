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

#define GPS_RX_PIN 44 //UART0
#define GPS_TX_PIN 43
#define GPS_BAUD 9600

#define MAX_ULTRASONIC_DISTANCE 200 // Maximum distance in cm

#define HOSTNAME "esp32-data"

camera_fb_t* fb = NULL; // Frame buffer pointer

const int imageWidth = 96;
const int imageHeight = 96;
const int bufferLength = imageWidth * imageHeight;
uint8_t currentRawImageCopy[imageWidth * imageHeight];
SemaphoreHandle_t FrameMutex;

int previousLabelX = -1;
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
const byte TRIGGER_PIN = D11;
const byte ECHO_PIN = D12;
const int OBSTACLE_THRESHOLD_CM = 3; 

unsigned long now = millis();
unsigned long previousLcdPrintTime = 0;
int16_t gyroX, gyroY, gyroZ;
float angleX = 0.0, angleY = 0.0, angleZ = 0.0;
unsigned long previousTimeGyro = 0;
long gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float startingAbsoluteAngleZ = 0.0;
bool shouldBeMoving = false; 


const byte ACCEL_ZOUT_H = 0x3F;
float accelZ;
const float ACCEL_SENSITIVITY = 16384.0;

const char* mdnsName = "esp32s3-data";
bool isStreaming = false;

// Simulated variables 
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
DFRobot_AXP313A Axp; 
WiFiMulti WifiMulti;
TinyGPSPlus Gps;
HardwareSerial GpsSerial(0); 
Adafruit_MCP23X17 Mcp;
LiquidCrystal_I2C Lcd(LCD_I2C_ADRESS, 16, 2); 

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

  shouldBeMoving = true; 
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

  shouldBeMoving = true; 
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

  shouldBeMoving = true; 
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

  shouldBeMoving = true; 
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

  shouldBeMoving = false; 
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

void updateLEDsAutoMode() {
  int dir1_state = Mcp.digitalRead(DIR1_PIN);
  int dir2_state = digitalRead(DIR2_PIN);
  int pwm1_speed = analogRead(PWM1_PIN);
  int pwm2_speed = analogRead(PWM2_PIN);

  shouldRedLedsBlink = false;
  digitalWrite(RED_LED_PIN, LOW);
  shouldWhiteLedsBlink = false;
  digitalWrite(WHITE_LED_PIN, LOW);
  shouldLeftOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN1, LOW);
  shouldRightOrangeLedBlink = false;
  Mcp.digitalWrite(ORANGE_LED_PIN2, LOW);

  if (pwm1_speed == 0 && pwm2_speed == 0) {
    shouldRedLedsBlink = true;
    digitalWrite(RED_LED_PIN, HIGH);
  } else if (dir1_state == HIGH && dir2_state == HIGH) {
    shouldWhiteLedsBlink = true;
    digitalWrite(WHITE_LED_PIN, HIGH);
  } else if (dir1_state == LOW && dir2_state == LOW) {
      if (pwm1_speed > pwm2_speed + 30) { 
      shouldRightOrangeLedBlink = true;
      Mcp.digitalWrite(ORANGE_LED_PIN2, HIGH);
    } else if (pwm2_speed > pwm1_speed + 30) { 
      shouldLeftOrangeLedBlink = true;
      Mcp.digitalWrite(ORANGE_LED_PIN1, HIGH);
    }
    }

  }
  
  

  
  void handleObstacle() {
  if (ultrasonicDistance <= OBSTACLE_THRESHOLD_CM) {
    if (!obstacleDetected) {
      wasInAutoMode = !manualMode;
      obstacleDetected = true;
      backward(180); 

    }
  } else {
    if (obstacleDetected) {
      obstacleDetected = false;
      
      if (wasInAutoMode) {
        automaticMode();       
        Mcp.digitalWrite(DIR1_PIN, LOW);
        digitalWrite(DIR2_PIN, LOW);
        analogWrite(PWM1_PIN, currentSpeed);
        analogWrite(PWM2_PIN, currentSpeed);
      } else {
        stop(); 
        
      }
    }
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
// HTML and Javascript for the webpage
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

void handleServerImageCapture() {
  Server.send_P(200, "application/octet-stream", (const char*)currentRawImageCopy, (size_t)bufferLength);
}

void handleServerData() {
  String Json = "{";
  Json += "\"x\":" + String(previousLabelX) + ",";
  Json += "\"y\":" + String(previousLabelY) + ",";
  Json += "\"label\":\"" + previousLabel + "\"";
  Json += "}";
  Server.send(200, "application/json", Json);
}

static int convertBufferForAI(size_t offset, size_t length, float* targetBuffer) {
    for (size_t i = 0; i < length; i++) {
        targetBuffer[i] = (float)((int16_t)currentRawImageCopy[offset + i] - 128);
    }
    return 0;
}

void inferenceTask(void *pvParameters) {
  while (1) {
    if (xSemaphoreTake(FrameMutex, pdMS_TO_TICKS(500))) {
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // Expected: 96*96 = 9216
        signal.get_data = &convertBufferForAI; // Applies pixel - 128
        
        ei_impulse_result_t result = {0}; // Initialize result struct

        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);

        if (res == EI_IMPULSE_OK) {
          Serial.printf("Inference done (%d ms).\n", result.timing.classification);
          
          bool foundObject = false;
          // Iterate through all bounding boxes 
          for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
            auto bb = result.bounding_boxes[ix];
            // Check if the bounding box is valid and meets the confidence threshold
            if (bb.value > 0.2f) { 
              Serial.printf("  DETECTED: %s (Conf: %.3f) [ x: %u, y: %u, w: %u, h: %u ]\n", 
                                  bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
              previousLabelX = bb.x;
              previousLabelY = bb.y;
              previousLabel = String(bb.label);

              
              foundObject = true;
              // Break after the first detection above the threshold (or remove break to show all)
              break; 
            }
          }
          if (!foundObject) {
            Serial.println("  No objects detected above threshold.");  
            previousLabelX = -1; 
            previousLabelY = -1;
            previousLabel = "Searching..."; 
          }
        } else {
            // Log errors from the classifier
            Serial.printf("run_classifier failed with error: %d\n", res);
            previousLabelX = -1; 
            previousLabelY = -1;
            previousLabel = "Error"; 
        }
        
        xSemaphoreGive(FrameMutex);
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

  Mcp.pinMode(FLAME_SENSOR_PIN, INPUT);
  Mcp.pinMode(SMOKE_PIN, INPUT);
  Mcp.pinMode(BUZZER_PIN, OUTPUT);

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

  while (WifiMulti.run() != WL_CONNECTED) { 
    delay(500); 
    Serial.println("Wifi init failed!"); 
  }
  MDNS.begin(HOSTNAME);
  Server.on("/", HTTP_GET, handleServerRoot);
  Server.on("/capture", HTTP_GET, handleServerImageCapture);
  Server.on("/data", handleServerData);
  Server.begin();

  FrameMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(inferenceTask, "Inference", 16384, NULL, 1, NULL, 1); //task function name, name for debugging, stack size in bytes, inputs, priority, handle

  xTaskCreate(ultrasonicTask, "Ultrasonic", 2048, NULL, 1, NULL);

  Lcd.setCursor(0, 0);
  Lcd.print("Bus Departed");
  departure = true;

  //Mcp.digitalWrite(BUZZER_PIN, HIGH);
  //delay(500);
  //Mcp.digitalWrite(BUZZER_PIN, LOW);
}

void loop() {
  now = millis();
  Server.handleClient(); 

  camera_fb_t * fb = esp_camera_fb_get();
  if (fb) {
    if (fb->len == bufferLength) { // Only copy if it matches 9216
      if (xSemaphoreTake(FrameMutex, pdMS_TO_TICKS(50))) {
        memcpy(currentRawImageCopy, fb->buf, fb->len);
        xSemaphoreGive(FrameMutex);
      }
    } else {
      // If this prints, your camera is not outputting 96x96!
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
    }
  }
  
  checkRedLeds();
  checkWhiteLeds();
  checkLeftOrangeLed();
  checkRightOrangeLed();

  // Feed characters from GPS to TinyGPS++
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
    if (shouldBeMoving && !obstacleDetected) {
      pidCorrection(targetAngle, angleZ, currentSpeed);
    } else {
      if (!obstacleDetected) {
        stop(); 
      }
    }
    updateLEDsAutoMode();
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
    if (!obstacleDetected){
      shouldBeMoving = false;
    }    
  } else if (ultrasonicDistance < 500.0f) {
    if (!obstacleDetected) {
      shouldBeMoving = true;
    }
    
  }

  Serial.println((String)"Ultrasonic:" + ultrasonicDistance);

  delay(1);
}
