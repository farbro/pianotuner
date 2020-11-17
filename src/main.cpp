#define BRAKE_PIN 12
#define MOTOR_PIN 13
#define LOOP_PERIOD 10

#include <Arduino.h>
#include <ESP32Servo.h>
#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>

#include "config.h"

AsyncWebServer server(80);

Servo brake;
Servo motor;

typedef struct {
  int motor_speed = 50;
  int motor_acceleration = 1000;
  int brake_speed = 100;
  int motor_start_speed = 0;
  int brake_delay = 3000;
  int release_delay = 2000;
} RunParams;

enum STATES {
  RESET,
  IDLE,
  RUN,
  ACCELERATE,
  BRAKE
};


int state = RESET;
unsigned long start_time = 0, brake_time = 0;
RunParams params;

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void updateParam(int *param, const char* key, AsyncWebServerRequest *request) {
  if (request->hasParam(key)) {
    *param = request->getParam(key)->value().toInt();
  }
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Send a GET request to <IP>/get?message=<message>
    server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {
        RunParams new_params;

        updateParam(&(params.motor_speed), "motorSpeed", request);
        updateParam(&(params.motor_start_speed), "motor_start_speed", request);
        updateParam(&(params.motor_acceleration), "motorAcceleration", request);
        updateParam(&(params.brake_speed), "brakeSpeed", request);
        updateParam(&(params.brake_delay), "brakeDelay", request);
        updateParam(&(params.release_delay), "releaseDelay", request);

        if (state == IDLE) {

          state = RUN;
          params = new_params;
        }

        request->send(200, "text/plain", "OK");
    });

  server.onNotFound(notFound);

  server.begin();

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  brake.attach(BRAKE_PIN);
  motor.attach(MOTOR_PIN);
}

void loop() {
  int t;
  int motor_speed, brake_position;

  switch (state) {
    case RESET:
      motor.write(0);
      brake.write(0);

      state = IDLE; break;

    case IDLE:
      break;

    case RUN:
      start_time = millis();

      state = ACCELERATE;
      break;

    case ACCELERATE:
      t = millis() - start_time;
      motor_speed = constrain(params.motor_start_speed + t/1000.0*params.motor_acceleration, 0, 100);
      motor.write(map(motor_speed, 0, 100, 0, 180));

      if (t >= params.brake_delay) {
        motor.write(0);
        brake_time = millis();
        state = BRAKE;
      }
      break;

    case BRAKE:
      t = millis() - brake_time;
      
      brake_position = constrain(t/1000.0*params.brake_speed, 0, 100);
      brake.write(map(brake_position, 0, 100, 0, 180));
      
      if (t >= params.release_delay) {
        state = RESET;
      }
      break;
  }

  delay(LOOP_PERIOD);
}

