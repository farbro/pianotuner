

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
  float motor_acceleration = 5; // rad/s²

  float brake_holding_duration = 0.3; // s

  float motor_standstill = 1500; // ppm µs period
  float motor_forward_max = 2000; // ppm µs period
  float motor_backward_max = 1000; // ppm µs period
  float motor_control_period = 100; // ms
  float motor_pid_kp = 15;
  float motor_pid_ki = 0.5;
  float motor_pid_kd = 0;
  float motor_max_error = 0.1; // rad/s

  float in_target_duration = 0.2; // s

  float brake_speed = -100; // fraction/s
  float brake1_center = 90; // deg
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
  ACCELERATE,
  BRAKE,
  HOLD
};


int state = RESET;
unsigned long t0 = 0;
unsigned long last_run = 0;
int direction = 1;
RunParams params;

unsigned long maxCycleTime = 0;
unsigned long minCycleTime = LONG_MAX;
unsigned long avgCycleTime = 0;
unsigned long lastCycle = 0;
unsigned int cycles = 0;
unsigned long lastSpeedReport = 0;

unsigned long lastMotorRevTime = 0;
bool lastMotorRevValue = 0;
float targetMotorSpeed = 0;
float currentMotorSpeed = 0;
float lastMotorError = 0;
float rampingMotorSpeed = 0;
unsigned long lastMotorControl = 0;
int num_attempts = 0;

unsigned long lastOffTarget = 0;

float motorPID_i = 0;


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
void setMotor(float v) {

  #ifdef REVERSE_MOTOR
    v *= -1;
  #endif
  
  targetMotorSpeed = v;

}

void controlMotor() {
  unsigned long period = millis() - lastMotorControl;

  if (targetMotorSpeed == 0) {
    motor.write(params.motor_standstill);
    rampingMotorSpeed = 0;
    lastMotorError = 0;
    motorPID_i = 0;
    return;
  }

  if (period > params.motor_control_period) {
    lastMotorControl = millis();

    // Ramp the speed
    float rampingError = abs(targetMotorSpeed) - rampingMotorSpeed;
    float maxAcceleration = params.motor_acceleration*params.motor_control_period/1000;
    rampingMotorSpeed += constrain(rampingError, -maxAcceleration, maxAcceleration);

    float error = abs(rampingMotorSpeed) - currentMotorSpeed;
    
    float P = params.motor_pid_kp*error;

    motorPID_i += error;
    float I = params.motor_pid_ki*motorPID_i;
    float D = 0;

    lastMotorError = error;

    float output =  P + I + D;

    if (targetMotorSpeed < 0)
      output *= -1;

    float targetMotorPPM = constrain(params.motor_standstill + output, params.motor_backward_max, params.motor_forward_max);


    motor.writeMicroseconds(targetMotorPPM);
    //Serial.printf("rampingMotorSpeed=%f\ttargetMotorSpeed=%f\trampingError=%f\tP=%f\tI=%f\tD=%f\tppm=%f\n", rampingMotorSpeed, targetMotorSpeed, rampingError, P, I, D, targetMotorPPM);
  
  }
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
        updateParam(&(new_params.motor_acceleration), "motor_acceleration", request);
        updateParam(&(new_params.motor_standstill), "motor_standstill", request);
        updateParam(&(new_params.motor_forward_max), "motor_forward_max", request);
        updateParam(&(new_params.motor_backward_max), "motor_backward_max", request);

        updateParam(&(new_params.motor_pid_ki), "motor_pid_ki", request);
        updateParam(&(new_params.motor_pid_kp), "motor_pid_kp", request);
        updateParam(&(new_params.motor_max_error), "motor_max_error", request);

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
          Serial.printf("Setting parameter %s = %f\n", "motor", targetMotorSpeed);
        }

        if (request->hasParam("motor_pid_ki")) {
          value = request->getParam("motor_pid_ki")->value().toFloat();
          params.motor_pid_ki = value;
          Serial.printf("Setting parameter %s = %f\n", "motor_pid_ki", value);
        }

        if (request->hasParam("motor_pid_kp")) {
          value = request->getParam("motor_pid_kp")->value().toFloat();
          params.motor_pid_kp = value;
          Serial.printf("Setting parameter %s = %f\n", "motor_pid_kp", value);
        }

        if (request->hasParam("motor_pid_kd")) {
          value = request->getParam("motor_pid_kd")->value().toFloat();
          params.motor_pid_kd = value;
          Serial.printf("Setting parameter %s = %f\n", "motor_pid_kd", value);
        }

        if (request->hasParam("motor_acceleration")) {
          value = request->getParam("motor_acceleration")->value().toFloat();
          params.motor_acceleration = value;
          Serial.printf("Setting parameter %s = %f\n", "motor_acceleration", value);
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
  motor.attach(MOTOR_PIN);

  pinMode(MOTOR_REV_PIN, INPUT);

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
  lastMotorRevValue = digitalRead(MOTOR_REV_PIN);
}

void loop() {
  unsigned long t;
  float brake_position;

  // Meaure cycle time
  long cycleTime = micros() - lastCycle;
  lastCycle = micros();

  if (cycleTime > maxCycleTime)
    maxCycleTime = cycleTime;

  if (cycleTime < minCycleTime)
    minCycleTime = cycleTime;
  
  avgCycleTime += cycleTime;

  if (cycles >= CYCLE_TIME_REPORT_INTERVAL) {
    avgCycleTime /= cycles;
    cycles = 0;
    
    //Serial.printf("Avg cycle time:\t%ld\n", avgCycleTime);
    //Serial.printf("Max cycle time:\t%ld\n", maxCycleTime);
    //Serial.printf("Min cycle time:\t%ld\n", minCycleTime);

    avgCycleTime = 0;
    maxCycleTime = 0;
    minCycleTime = LONG_MAX;
  }
  cycles++;

  // Calculate motor speed
  bool currentMotorRevValue = digitalRead(MOTOR_REV_PIN);

  unsigned long motorRevTime = micros() - lastMotorRevTime;

  if (!lastMotorRevValue && currentMotorRevValue) {
    lastMotorRevTime = micros();

    float newMotorSpeed = 1.0/(motorRevTime*10.0e-6) * 2*3.1419; // rad/s
    currentMotorSpeed = currentMotorSpeed*(1 - MOTOR_AVG_RATIO) + newMotorSpeed*MOTOR_AVG_RATIO;
  }

  if (motorRevTime > 1e6)
    currentMotorSpeed = 0;

  lastMotorRevValue = currentMotorRevValue;

  // Report speed
  if (millis() - lastSpeedReport > 1000) {
    lastSpeedReport = millis();
    Serial.printf("Motor target = %f\t current = %f\t ppm = %d\n", targetMotorSpeed, currentMotorSpeed, motor.readMicroseconds());
  }

  // Regulate motor speed
  controlMotor();

  // Run state machine
  if (millis() - last_run > LOOP_PERIOD) {

    last_run = millis();
    t = millis() - t0;

    switch (state) {
      case RESET:
        Serial.println("Resetting...");

        setMotor(0);
        setBrake1(0);
        setBrake2(0);

        state = IDLE;
        Serial.println("RESET -> IDLE");
        t0 = millis();
        num_attempts = 0;

      case IDLE:
        break;

      case RUN:
        t0 = millis();
        Serial.printf("Run #%d\n", num_attempts);
        num_attempts++;

        state = ACCELERATE;
        Serial.println("RUN -> ACCELERATE");
        if (params.motor_speed >= 0) {
          direction = 1;
        } else {
          direction = -1;
        }
        setMotor(params.motor_speed);
        break;

      case ACCELERATE:
        //Serial.printf("error=%f\t", abs(targetMotorSpeed) - currentMotorSpeed);
        //Serial.printf("off target=%ld\n", millis() - lastOffTarget);

        if (abs(targetMotorSpeed) - currentMotorSpeed > params.motor_max_error) {
          lastOffTarget = millis();
        } else if (millis() - lastOffTarget >= params.in_target_duration*1000) {
          t0 = millis();
          state = BRAKE;
          Serial.println("ACCELERATE -> BRAKE");
          setMotor(0);
        }

        if (t > ACCELERATION_TIMEOUT*1000 && currentMotorSpeed < 1) {
          if (num_attempts <= 3) {
            Serial.printf("Timeout, retry %d\n", num_attempts);
            Serial.println("ACCELERATE -> RUN");
            setMotor(0);
            state = RUN;
          } else {
            Serial.printf("Timeout, aborting after %d times \n", num_attempts);
            Serial.println("ACCELERATE -> RESET");
            state = RESET;
          }
        }
        break;


      case BRAKE:
        brake_position = t/1000.0*params.brake_speed*direction;
        setBrake1(brake_position);
        setBrake2(brake_position);
        
        if (brake_position >= 1 || brake_position <= -1) {
          t0 = millis();
          state = HOLD;
          Serial.println("BRAKE -> HOLD");
        }
        break;

      case HOLD:
        if (t >= params.brake_holding_duration*1000) {
          state = RESET;

          Serial.println("HOLD -> RESET");
        }
        
    }
  }

  ArduinoOTA.handle();
}

