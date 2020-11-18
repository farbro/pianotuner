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
  int brake_delay = 1000;
  int release_delay = 2000;
} RunParams;

enum STATES {
  RESET,
  IDLE,
  RUN,
  ACCELERATE,
  WAIT,
  BRAKE
};


int state = RESET;
unsigned long t0 = 0;
RunParams params;

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void updateParam(int *param, const char* key, AsyncWebServerRequest *request) {
  if (request->hasParam(key)) {
    *param = request->getParam(key)->value().toInt();
    Serial.printf("Setting parameter %s = %d\n", key, *param);
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

        Serial.println("Run requested");

        RunParams new_params;

        updateParam(&(new_params.motor_speed), "motorSpeed", request);
        updateParam(&(new_params.motor_start_speed), "motor_start_speed", request);
        updateParam(&(new_params.motor_acceleration), "motorAcceleration", request);
        updateParam(&(new_params.brake_speed), "brakeSpeed", request);
        updateParam(&(new_params.brake_delay), "brakeDelay", request);
        updateParam(&(new_params.release_delay), "releaseDelay", request);


        if (state == IDLE) {

          Serial.println("Starting run...");
          Serial.printf("motor_speed = %d\n", new_params.motor_speed);
          Serial.printf("motor_start_speed = %d\n", new_params.motor_start_speed);
          Serial.printf("motor_acceleration = %d\n", new_params.motor_acceleration);
          Serial.printf("motor_brake_speed = %d\n", new_params.brake_speed);
          Serial.printf("motor_speed = %d\n", new_params.motor_speed);
          Serial.printf("brake_delay = %d\n", new_params.brake_delay);
          Serial.printf("release_delay = %d\n", new_params.release_delay);

          state = RUN;
          params = new_params;
        }

        request->send(204);
    });

  server.onNotFound(notFound);

  server.begin();

  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  brake.attach(BRAKE_PIN, BRAKE_MIN, BRAKE_MAX);
  motor.attach(MOTOR_PIN, MOTOR_MIN, MOTOR_MAX);
}

void loop() {
  int t;
  int motor_speed, brake_position;

  t = millis() - t0;

  switch (state) {
    case RESET:
      Serial.println("RESET");

      motor.write(0);
      brake.write(0);

      state = IDLE; break;
      Serial.println("IDLE");

    case IDLE:
      break;

    case RUN:
      t0 = millis();

      state = ACCELERATE;
        Serial.println("ACCELERATE");
      break;

    case ACCELERATE:
      t = millis() - t0;
      motor_speed = constrain(params.motor_start_speed + t/1000.0*params.motor_acceleration, 0, params.motor_speed);
      motor.write(map(motor_speed, 0, 100, 0, 180));

      Serial.printf("motor_speed = %d\n", motor.read());

      if (motor_speed >= params.motor_speed) {
        t0 = millis();
        state = WAIT;
        Serial.println("WAIT");
      }
      break;

    case WAIT:
      if (t >= params.brake_delay) {
        motor.write(0);
        t0 = millis();
        state = BRAKE;
        Serial.println("BRAKE");
      }
      break;

    case BRAKE:
      
      brake_position = constrain(t/1000.0*params.brake_speed, 0, 100);
      brake.write(map(brake_position, 0, 100, 0, 180));

      Serial.printf("brake_position = %d\n", brake.read());
      
      if (t >= params.release_delay) {
        state = RESET;

        Serial.println("RESET");
      }
      break;
  }

  delay(LOOP_PERIOD);
}

