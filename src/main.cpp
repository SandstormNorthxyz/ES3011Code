#include <Arduino.h>
#include <Telemetry.h>
#include "smartmotor.h"

float dt = 0;
long last_update_time = 0;
int test = 0;

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
const int32_t ENCODER_TICKS_PER_SHAFT_REV= ENCODER_TICKS_PER_REV * GEAR_RATIO;
#define DELAY_PERIOD 0

// INIT SMART MOTORS
SmartMotor leftMotor = 0x0A; // INIT MOTOR W/ DEFAULT ADDRESS
SmartMotor rightMotor = 0x0B;
uint16_t leftStatus = 0;
uint16_t rightStatus = 0;
float leftRPM = 0;
float rightRPM = 0;
uint32_t leftPos = 0;
uint32_t rightPos = 0;
float leftSetpoint = 0;
float rightSetpoint = 0;
// SmartMotor motors[] = {0x05,0x06,0x07}; // INIT MOTOR W/ DEFAULT ADDRESS

//vec2<float> pos;
//angle dir;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Telemetry::init();

  
  Telemetry::initPacket(Telemetry::loopTime, &dt);
  Telemetry::initPacket(Telemetry::test, &test);

  Telemetry::initPacket(Telemetry::leftStatus, &leftStatus);
  Telemetry::initPacket(Telemetry::rightStatus, &rightStatus);

  Telemetry::initPacket(Telemetry::leftRPM, &leftRPM);
  Telemetry::initPacket(Telemetry::rightRPM, &rightRPM);
  Telemetry::initPacket(Telemetry::leftPos, &leftPos);
  Telemetry::initPacket(Telemetry::rightPos, &rightPos);

  Telemetry::initPacket(Telemetry::leftSetpoint, &leftSetpoint);
  Telemetry::initPacket(Telemetry::rightSetpoint, &rightSetpoint);
  
}

void loop() {
  dt = (float)(micros() - last_update_time) / 1.0E6;
  last_update_time = micros(); // use microseconds because the main loop is running in 3 - 5 milliseconds
  // Serial.println("Hello");
  // delay(20);

  leftStatus = leftMotor.set_rpm(leftSetpoint);
  rightStatus = rightMotor.set_rpm(rightSetpoint);
  
  leftRPM = leftMotor.get_rpm();
  rightRPM = rightMotor.get_rpm();
  leftPos = leftMotor.get_position();
  rightPos = rightMotor.get_position();


  Telemetry::sendPacket(Telemetry::loopTime);
  Telemetry::sendPacket(Telemetry::test);
  Telemetry::sendPacket(Telemetry::leftStatus);
  Telemetry::sendPacket(Telemetry::rightStatus);
  Telemetry::sendPacket(Telemetry::leftRPM);
  Telemetry::sendPacket(Telemetry::rightRPM);
  Telemetry::sendPacket(Telemetry::leftPos);
  Telemetry::sendPacket(Telemetry::rightPos);
  Telemetry::update();

  test++;

  delay(20);
}