

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
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "config.h"

AsyncWebServer server(80);

Servo brake1;
Servo brake2;
Servo motor;

typedef struct {
  float motor_speed = 0.5; // fraction/s
  float motor_acceleration = 0.1; // fraction/s^2
  
  float motor_warmup_duration = 2; // s

  float motor_forward_stall = 65; // 0 - 180
  float motor_standstill = 0;
  float motor_forward_max = 170;

  float motor_maxspeed_duration = 4; // s
  float freespin_duration = 0; // s
  float brake_holding_duration = 0.3; // s
  float motor_warmup_duration = 2; // s

  float motor_forward_stall = 99; // 0 - 180
  float motor_backward_stall = 81; // 0 - 180
  float motor_standstill = 90;
  float motor_forward_max = 180;
  float motor_backward_max = 55;

  float motor_maxspeed_duration = 4; // s
  float freespin_duration = 0; // s
  float brake_holding_duration = 0.3; // s

  float brake_speed = -1; // fraction/s
  float brake1_center = 97; // deg
  float brake1_min = 0; // deg
  float brake1_max = 180; // deg

  float brake2_center = 90; // deg
  float brake2_min = 0; // deg
  float brake2_max = 180;// deg
} RunParams;

enum STATES {
  RESET,
  IDLE,
  RUN,
  STARTUP,
  ACCELERATE,
  MAXSPEED,
  FREESPIN,
  BRAKE,
  HOLD
};


int state = RESET;
unsigned long t0 = 0;
unsigned long last_run = 0;
int direction = 1;
RunParams params;

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void updateParam(float *param, const char* key, AsyncWebServerRequest *request) {
  if (request->hasParam(key)) {
    *param = request->getParam(key)->value().toFloat();
    Serial.printf("Setting parameter %s = %f\n", key, *param);
  }
}

// Set motor value (-1 to 1))
void setMotor(float value) {

  value = constrain(value, -1, 1);

  float raw = params.motor_standstill;

  if (value > 0)
    raw = params.motor_forward_stall + value*(params.motor_forward_max - params.motor_forward_stall);
  else if (value < 0)
    raw = params.motor_backward_stall - value*(params.motor_backward_stall - params.motor_backward_max);

  Serial.printf("motor = %f (%f)\n", value, raw);
  motor.write(raw);
}

// Set brake1 value (-1 to 1)
void setBrake1(float value) {

  value = constrain(value, -1, 1);

  float raw = params.brake1_center;

  if (value > 0)
    raw += value*(params.brake1_max - params.brake1_center);
  else if (value < 0)
    raw += value*(params.brake1_center - params.brake1_min);

  Serial.printf("brake1 = %f (%f)\n", value, raw);
  brake1.write(raw);
}

// Set brake2 value (-1 to 1)
void setBrake2(float value) {

  value = constrain(value, -1, 1);

  float raw = params.brake2_center;

  if (value > 0)
    raw += value*(params.brake2_max - params.brake2_center);
  else if (value < 0)
    raw += value*(params.brake2_center - params.brake2_min);

  Serial.printf("brake2 = %f (%f)\n", value, raw);
  brake2.write(raw);
}

void setup() {

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  WiFi.setHostname(HOSTNAME);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.printf("WiFi Failed!\n");
      return;
  }

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(WiFi.getHostname());

  // Send a GET request to <IP>/get?message=<message>
    server.on("/run", HTTP_GET, [] (AsyncWebServerRequest *request) {

        Serial.println("Run requested");

        RunParams new_params;

        updateParam(&(new_params.motor_speed), "motor_speed", request);
        updateParam(&(new_params.motor_warmup_duration), "motor_warmup_duration", request);
        updateParam(&(new_params.motor_acceleration), "motor_acceleration", request);
        updateParam(&(new_params.motor_forward_stall), "motor_forward_stall", request);
        updateParam(&(new_params.motor_backward_stall), "motor_backward_stall", request);
        updateParam(&(new_params.motor_standstill), "motor_standstill", request);
        updateParam(&(new_params.motor_forward_max), "motor_forward_max", request);
        updateParam(&(new_params.motor_backward_max), "motor_backward_max", request);
        updateParam(&(new_params.freespin_duration), "freespin_duration", request);

        updateParam(&(new_params.motor_maxspeed_duration), "motor_maxspeed_duration", request);
        updateParam(&(new_params.brake_speed), "brake_speed", request);
        updateParam(&(new_params.brake_holding_duration), "brake_holding_duration", request);

        updateParam(&(new_params.brake1_center), "brake1_center", request);
        updateParam(&(new_params.brake1_min), "brake1_min", request);
        updateParam(&(new_params.brake1_max), "brake1_max", request);

        updateParam(&(new_params.brake2_center), "brake2_center", request);
        updateParam(&(new_params.brake2_min), "brake2_min", request);
        updateParam(&(new_params.brake2_max), "brake2_max", request);

        if (state == IDLE) {

          Serial.println("Starting run...");

          state = RUN;
          params = new_params;
        }

        request->send(204);
    });


    // Set outputs (for testing)
    server.on("/set", HTTP_GET, [] (AsyncWebServerRequest *request) {
        Serial.println("Setting outputs");

        int raw = 0;
        float value;

        if (request->hasParam("raw")) raw = 1;

        if (request->hasParam("brake1")) {
          value = request->getParam("brake1")->value().toFloat();
          if (raw)
            brake1.write(value);
          else
            setBrake1(value);

          Serial.printf("Setting parameter %s = %d\n", "brake1", brake1.read());
        }

        if (request->hasParam("brake2")) {
          value = request->getParam("brake2")->value().toFloat();
          if (raw)
            brake2.write(value);
          else
            setBrake2(value);
          Serial.printf("Setting parameter %s = %d\n", "brake2", brake2.read());
        }

        if (request->hasParam("motor")) {
          value = request->getParam("motor")->value().toFloat();
          if (raw)
            motor.write(value);
          else
            setMotor(value);
          Serial.printf("Setting parameter %s = %d\n", "motor", motor.read());
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

  brake1.attach(BRAKE1_PIN, BRAKE1_MIN, BRAKE1_MAX);
  brake2.attach(BRAKE2_PIN, BRAKE2_MIN, BRAKE2_MAX);
  motor.attach(MOTOR_PIN, MOTOR_MIN, MOTOR_MAX);


  ArduinoOTA
    .setMdnsEnabled(false)
    .setPort(OTA_PORT)
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void loop() {
  unsigned long t;
  float motor_speed, brake_position;

  if (millis() - last_run > LOOP_PERIOD) {
    last_run = millis();
    t = millis() - t0;

    switch (state) {
      case RESET:
        Serial.println("RESET");

        setMotor(0);
        setBrake1(0);
        setBrake2(0);

        state = IDLE; break;
        Serial.println("IDLE");

      case IDLE:
        break;

      case RUN:
        t0 = millis();

        state = STARTUP;
        Serial.println("ACCELERATE");
        if (params.motor_speed >= 0) {
          direction = 1;
        } else {
          direction = -1;
        }
        break;

      case STARTUP:
        if (t >= params.motor_warmup_duration*1000) {
          state = ACCELERATE;
          t0 = millis();
        }
        break;

      case ACCELERATE:
        motor_speed = t/1000.0*params.motor_acceleration;
        setMotor(motor_speed*direction);

        if (motor_speed >= abs(params.motor_speed)) {
          t0 = millis();
          state = MAXSPEED;
          Serial.println("WAIT");
        }
        break;

      case MAXSPEED:
        if (t >= params.motor_maxspeed_duration*1000) {
          setMotor(0);
          t0 = millis();
          state = FREESPIN;
          Serial.println("FREESPIN");
        }
        break;

      case FREESPIN:
        if (t >= params.freespin_duration*1000) {
          t0 = millis();
          state = BRAKE;
          Serial.println("BRAKE");
        }
        break;


      case BRAKE:
        brake_position = t/1000.0*params.brake_speed*direction;
        setBrake1(brake_position);
        setBrake2(brake_position);
        
        if (brake_position >= 1 || brake_position <= -1) {
          t0 = millis();
          state = HOLD;
          Serial.println("HOLD");
        }
        break;

      case HOLD:
        if (t >= params.brake_holding_duration*1000) {
          state = RESET;

          Serial.println("RESET");
        }
        
    }
  }

  ArduinoOTA.handle();
}

