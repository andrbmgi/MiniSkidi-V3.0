/* ProfessorBoots
   John Cheroske 1/6/2024
   MiniSkidi 3.0

   Thank you to the following people for contributing to this sketch
   -TomVerr99 "Excellent Job organizing what was a very messy intial sketch"
   -CrabRC "I dont even know where to start, but thank you for making numerous improvemnts/suggestions
   across both mechanical designs and software."
   -Fortinbra "Always willing to provide the discord group with a good meme or two, as well as lend a helping hand
   in multiple ways."

  Some tidbits to check

  -Install the esp32 boards manager into the arduino IDE"
  Programming Electronics Academy has a good tutorial: https://youtu.be/py91SMg_TeY?si=m1OWPBPlK-QHJ2Xx"
  -Select "ESP32 Dev Module" under tools>Board>ESP32 Arduino before uploading sketch.
  -The following include statements with comments "by -----" are libraries that can be installed
  directly inside the arduino IDE under Sketch>Include Library>Manage Libraries
*/
#include <Arduino.h>

#include <ESP32Servo.h> // by Kevin Harrington
#include <ESPAsyncWebSrv.h> // by dvarrel
#include <iostream>
#include <sstream>
#include "sbus.h"

#if defined(ESP32)
#include <AsyncTCP.h> // by dvarrel
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESPAsyncTCP.h> // by dvarrel
#endif


// defines
#define SERIAL_TX_PIN    17
#define SERIAL_RX_PIN    16

#define bucketServoPin  23
#define auxServoPin 22
#define lightPin1 18
#define lightPin2 5
#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define ARMUP 5
#define ARMDOWN 6
#define STOP 0

#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0
#define ARM_MOTOR 2

#define FORWARD 1
#define BACKWARD -1

#define MIN_PULSE_WIDTH       500     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 

#define MS_CENTER 992
#define MS_MIN 172
#define MS_MAX 1811
#define DEAD_PERCENT 5
#define DEAD_CENTER 12
// global constants

#define AUX_CENTER 992
#define AUX_MIN 172
#define AUX_MAX 1811

extern const char* htmlHomePage PROGMEM;
const char* ssid     = "ProfBoots MiniSkidi OG";

// global variables

// PWM properties
const int freq = 50000;
const int resolution = 8;

Servo bucketServo;
Servo auxServo;

/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx(&Serial2, SERIAL_RX_PIN, SERIAL_TX_PIN, true, false);
/* SBUS data */
bfs::SbusData data;

bool horizontalScreen;//When screen orientation is locked vertically this rotates the D-Pad controls so that forward would now be left.
bool removeArmMomentum = false;
bool light = false;

struct MOTOR_PINS
{
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins =
{
  {25, 26},  //RIGHT_MOTOR Pins (IN1, IN2)
  {33, 32},  //LEFT_MOTOR  Pins
  {21, 19},  //ARM_MOTOR pins
};

std::vector<MOTOR_PINS> motorPinsPWMChannel =
{
  {14, 15},  //RIGHT_MOTOR PWM Channel
  {12, 13},  //LEFT_MOTOR  PWM Channel
  {10, 11},  //ARM_MOTOR PWM Channel
};

AsyncWebServer server(80);
AsyncWebSocket wsCarInput("/CarInput");

long map_ms_percent(int ms_value) {
  long percent_value = map(ms_value, MS_MIN, MS_MAX, -100, 100);
  return percent_value;
}

long map_ms_duty(int ms_value) {
  long percent_value = map(ms_value, MS_MIN, MS_MAX, -255, 255);
  return percent_value;
}

long map_aux(int ms_value) {
  long servo_value = map(ms_value, AUX_MIN, AUX_MAX, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  return servo_value;
}

void perform_remote_command(bfs::SbusData data) {
  long remote_throttle = map_ms_percent(data.ch[0]);
  long remote_steering = map_ms_percent(data.ch[1]);
  long remote_arm = map_ms_duty(data.ch[2]);
  long remote_bucket = map_aux(data.ch[3]);
  long remote_aux = map_aux(data.ch[4]);
  long remote_lights = map_ms_percent(data.ch[5]);

  // dead zone when sticks are centered
  if (remote_throttle > -DEAD_PERCENT && remote_throttle < DEAD_PERCENT) {
    remote_throttle = 0;
  }
  if (remote_steering > -DEAD_PERCENT && remote_steering < DEAD_PERCENT) {
    remote_steering = 0;
  }

  // input mixing
  int leftThrottle = remote_throttle + remote_steering; 
  int rightThrottle = remote_throttle - remote_steering;

  // normalize if max > 100
  float factor = 1.0;
  if (leftThrottle > 100) {
    factor = 100.0 / (float) leftThrottle;
    leftThrottle = round(factor * leftThrottle);
  }
  if (rightThrottle > 100) {
    factor = 100.0 / (float) rightThrottle;
    rightThrottle = round(factor * rightThrottle); 
  }

  // map to duty cycle
  leftThrottle = map( leftThrottle, -100, 100, -255, 255);
  rightThrottle = map( rightThrottle, -100, 100, -255, 255);

  //constrain
  leftThrottle = constrain(leftThrottle, -255, 255);
  rightThrottle = constrain(rightThrottle, -255, 255);


  if (leftThrottle > 0) {
    rotateMotor(LEFT_MOTOR, FORWARD, leftThrottle);
  } else if (leftThrottle < 0) {
    rotateMotor(LEFT_MOTOR, BACKWARD, abs(leftThrottle));
  } else {
    rotateMotor(LEFT_MOTOR, STOP, 0);
  }

  if (rightThrottle > 0) {
    rotateMotor(RIGHT_MOTOR, FORWARD, rightThrottle);
  } else if (rightThrottle < 0 - DEAD_CENTER) {
    rotateMotor(RIGHT_MOTOR, BACKWARD, abs(rightThrottle));
  } else {
    rotateMotor(RIGHT_MOTOR, STOP, 0);
  }

  if (remote_arm > 0 + DEAD_CENTER) {
    // Lower
    rotateMotor(ARM_MOTOR, FORWARD, remote_arm);
  } else if (remote_arm < 0 - DEAD_CENTER) {
    // Raise
    rotateMotor(ARM_MOTOR, BACKWARD, abs(remote_arm));
  } else {
    rotateMotor(ARM_MOTOR, STOP, 0);
  }

  bucketTilt(remote_bucket);
  auxControl(remote_aux);

  if (remote_lights > 0) {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
    light = true;
  } else {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
    light = false;
  }
}

void rotateMotor(int motorNumber, int motorDirection, uint32_t speed)
{
  if (motorDirection == FORWARD)
  { 
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN1, speed);
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN2, STOP);
  }
  else if (motorDirection == BACKWARD)
  {
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN1, STOP);
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN2, speed);
  }
  else
  {
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN1, STOP);
    ledcWrite(motorPinsPWMChannel[motorNumber].pinIN2, STOP);
  }
}

void moveCar(int inputValue, uint32_t speed)
{
  Serial.printf("Got value as %d\n", inputValue);
  if (!(horizontalScreen))
  {
    switch (inputValue)
    {

      case UP:
        rotateMotor(RIGHT_MOTOR, FORWARD, speed);
        rotateMotor(LEFT_MOTOR, FORWARD, speed);
        break;

      case DOWN:
        rotateMotor(RIGHT_MOTOR, BACKWARD, speed);
        rotateMotor(LEFT_MOTOR, BACKWARD, speed);
        break;

      case LEFT:
        rotateMotor(RIGHT_MOTOR, BACKWARD, speed);
        rotateMotor(LEFT_MOTOR, FORWARD, speed);
        break;

      case RIGHT:
        rotateMotor(RIGHT_MOTOR, FORWARD, speed);
        rotateMotor(LEFT_MOTOR, BACKWARD, speed);
        break;

      case STOP:
        rotateMotor(ARM_MOTOR, STOP, STOP);
        rotateMotor(RIGHT_MOTOR, STOP, STOP);
        rotateMotor(LEFT_MOTOR, STOP, STOP);
        break;

      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD, speed);
        break;

      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD, speed);
        removeArmMomentum = true;
        break;

      default:
        rotateMotor(ARM_MOTOR, STOP, STOP);
        rotateMotor(RIGHT_MOTOR, STOP, STOP);
        rotateMotor(LEFT_MOTOR, STOP, STOP);
        break;
    }
  } else {
    switch (inputValue)
    {
      case UP:
        rotateMotor(RIGHT_MOTOR, BACKWARD, speed);
        rotateMotor(LEFT_MOTOR, FORWARD, speed);
        break;

      case DOWN:
        rotateMotor(RIGHT_MOTOR, FORWARD, speed);
        rotateMotor(LEFT_MOTOR, BACKWARD, speed);
        break;

      case LEFT:
        rotateMotor(RIGHT_MOTOR, BACKWARD, speed);
        rotateMotor(LEFT_MOTOR, BACKWARD, speed);
        break;

      case RIGHT:
        rotateMotor(RIGHT_MOTOR, FORWARD, speed);
        rotateMotor(LEFT_MOTOR, FORWARD, speed);
        break;

      case STOP:
        rotateMotor(ARM_MOTOR, STOP, STOP);
        rotateMotor(RIGHT_MOTOR, STOP, STOP);
        rotateMotor(LEFT_MOTOR, STOP, STOP);
        break;

      case ARMUP:
        rotateMotor(ARM_MOTOR, FORWARD, speed);
        break;

      case ARMDOWN:
        rotateMotor(ARM_MOTOR, BACKWARD, speed);
        break;

      default:
        rotateMotor(ARM_MOTOR, STOP, STOP);
        rotateMotor(RIGHT_MOTOR, STOP, STOP);
        rotateMotor(LEFT_MOTOR, STOP, STOP);
        break;
    }
  }
}

void bucketTilt(int bucketServoValue)
{
  bucketServo.write(bucketServoValue);
}
void auxControl(int auxServoValue)
{
  auxServo.write(auxServoValue);
}
void lightControl()
{
  if (!light)
  {
    digitalWrite(lightPin1, HIGH);
    digitalWrite(lightPin2, LOW);
    light = true;
    Serial.println("Made it to lights");
  }
  else
  {
    digitalWrite(lightPin1, LOW);
    digitalWrite(lightPin2, LOW);
    light = false;
  }
}

void handleRoot(AsyncWebServerRequest *request)
{
  request->send_P(200, "text/html", htmlHomePage);
}

void handleNotFound(AsyncWebServerRequest *request)
{
  request->send(404, "text/plain", "File Not Found");
}

void onCarInputWebSocketEvent(AsyncWebSocket *server,
                              AsyncWebSocketClient *client,
                              AwsEventType type,
                              void *arg,
                              uint8_t *data,
                              size_t len)
{
  switch (type)
  {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      moveCar(STOP, STOP);
      break;
    case WS_EVT_DATA:
      AwsFrameInfo *info;
      info = (AwsFrameInfo*)arg;
      if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
      {
        std::string myData = "";
        myData.assign((char *)data, len);
        std::istringstream ss(myData);
        std::string key, value;
        std::getline(ss, key, ',');
        std::getline(ss, value, ',');
        Serial.printf("Key [%s] Value[%s]\n", key.c_str(), value.c_str());
        int valueInt = atoi(value.c_str());
        if (key == "MoveCar")
        {
          moveCar(valueInt, 255);
        }
        else if (key == "AUX")
        {
          auxControl(valueInt);
        }
        else if (key == "Bucket")
        {
          bucketTilt(valueInt);
        }
        else if (key == "Light")
        {
          lightControl();
        }
        else if (key == "Switch")
        {
          if (!(horizontalScreen))
          {
            horizontalScreen = true;
          }
          else {
            horizontalScreen = false;
          }
        }
      }
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
    default:
      break;
  }
}

void setUpPinModes()
{
  

  for (int i = 0; i < motorPins.size(); i++)
  {
    pinMode(motorPins[i].pinIN1, OUTPUT);
    pinMode(motorPins[i].pinIN2, OUTPUT);
    ledcSetup(motorPinsPWMChannel[i].pinIN1, freq, resolution);
    ledcSetup(motorPinsPWMChannel[i].pinIN2, freq, resolution);
    ledcAttachPin(motorPins[i].pinIN1, motorPinsPWMChannel[i].pinIN1);
    ledcAttachPin(motorPins[i].pinIN2, motorPinsPWMChannel[i].pinIN2);
  }

  moveCar(STOP, STOP);
  bucketServo.attach(bucketServoPin);
  auxServo.attach(auxServoPin);
  auxControl(150);
  bucketTilt(140);

  pinMode(lightPin1, OUTPUT);
  pinMode(lightPin2, OUTPUT);
}

void setup(void)
{
  setUpPinModes();
  Serial.begin(115200);

  /* Begin the SBUS communication */
  sbus_rx.Begin();

  WiFi.softAP(ssid );
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  server.on("/", HTTP_GET, handleRoot);
  server.onNotFound(handleNotFound);

  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);

  server.begin();
  Serial.println("HTTP server started");

}

void loop()
{
  /* Grab the received data */
  if (sbus_rx.Read()) {
    data = sbus_rx.data();
    perform_remote_command(data);
  }
  wsCarInput.cleanupClients();
}
