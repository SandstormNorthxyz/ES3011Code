#include <Arduino.h>
#include <Telemetry.h>
#include "MathUtils.h"
#include "smartmotor.h"

#include <stdlib.h>

#define USETELOMETER true

float dt = 0;
long last_update_time = 0;
int test = 0;

// MOTOR PROPERTIES
#define GEAR_RATIO 150           // MOTOR GEAR RATIO
#define ENCODER_TICKS_PER_REV 12 // NO. OF HIGH PULSES PER ROTATION
constexpr int32_t ENCODER_TICKS_PER_SHAFT_REV = ENCODER_TICKS_PER_REV * GEAR_RATIO;
constexpr float TRACK_WIDTH_WHEEL_TICKS = ENCODER_TICKS_PER_SHAFT_REV * 0.597529;
constexpr float CM_PER_TICK = 5.7 * 3.1415926535 / ENCODER_TICKS_PER_SHAFT_REV;

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

float P = 0.5;
float P1 = 3;
float P2 = 20;
float I = 0;
float D = 0.006;
// SmartMotor motors[] = {0x05,0x06,0x07}; // INIT MOTOR W/ DEFAULT ADDRESS

vec2<float> pos = {0, 0};
vec2<float> target = {0, 0};
angle dir = (angle){{0, 1}};

uint16_t fine = 0;

void update_odometry() {
  static float prevWheelAngle = 0;
  static vec2<int32_t> prevWheelTicks = {0, 0};
  vec2<int32_t> wheelTicks = {rightMotor.get_position(), -leftMotor.get_position()};


  float wheel_angle = (wheelTicks.x - wheelTicks.y) / TRACK_WIDTH_WHEEL_TICKS; //this could potentially be more fixed-point
  float delta_theta = wheel_angle - prevWheelAngle;
  prevWheelAngle = wheel_angle;


  vec2<int32_t> wheelDelta = (wheelTicks - prevWheelTicks);
  prevWheelTicks = wheelTicks;

  angle deltaA = MathUtils::angleFromRadians(delta_theta);
  float d = (wheelDelta.x + wheelDelta.y) / 2.0;
  d *= CM_PER_TICK;

  if (delta_theta == 0) {
    pos = pos + (dir).angle * d;
  } else {
    float r = d / MathUtils::getRadians(deltaA);
    pos = pos + MathUtils::rotate<vec2<float>>({r * deltaA.angle.y, r - r * deltaA.angle.x}, dir);
  }

  dir = dir + deltaA;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();

  #ifdef USETELOMETER
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

    Telemetry::initPacket(Telemetry::position, &pos);
    Telemetry::initPacket(Telemetry::targetPosition, &target);
    Telemetry::initPacket(Telemetry::heading, &dir);

    Telemetry::initPacket(Telemetry::aoirsetna, &fine);
    Telemetry::initPacket(Telemetry::P, &P);
    Telemetry::initPacket(Telemetry::I, &I);
    Telemetry::initPacket(Telemetry::D, &D);

    Telemetry::initPacket(Telemetry::P1, &P1);
    Telemetry::initPacket(Telemetry::P2, &P2);
  #endif

  leftMotor.reset();
  leftMotor.set_direction(1);
  rightMotor.reset();
}

bool driveToTarget() {
  vec2<float> error = target - pos;
  float lin = error * dir.angle;


  float angular = MathUtils::getRadians(dir- (angle){.angle = MathUtils::normalize(error)});

  if(MathUtils::length(error)< 0.5) {
    leftSetpoint = 0;
    rightSetpoint = 0;
    return true;
  }
  
  leftSetpoint  = lin * P1 + angular * P2;
  rightSetpoint = lin * P1 - angular * P2;


  return false;
}

bool turnToTarget() {
  vec2<float> error = target - pos;
  float lin = error * dir.angle;


  float angular = MathUtils::getRadians(dir - (angle){.angle = MathUtils::normalize(error)});

  if(abs(angular) < 0.01) {
    leftSetpoint = 0;
    rightSetpoint = 0;
    return true;
  }
  
  leftSetpoint  = angular * P2;
  rightSetpoint = angular * P2;

}

vec2<float> path[] = {{0, 2.54 * 12}, {2.54*12, 2.54*12}};

bool turning = false;
void loop() {
  dt = (float)(micros() - last_update_time) / 1.0E6;
  last_update_time = micros(); // use microseconds because the main loop is running in 3 - 5 milliseconds
  // Serial.println("Hello");
  // delay(20);

  leftMotor.tune_vel_pid(P, I, D);
  rightMotor.tune_vel_pid(P, I, D);

  leftRPM = leftMotor.get_rpm();
  rightRPM = rightMotor.get_rpm();
  leftPos = leftMotor.get_position();
  rightPos = rightMotor.get_position();

  target = path[fine];

  update_odometry();

  if(turning) {
    if(turnToTarget()) {
      turning = false; 
    }
  }
  else {
    if(driveToTarget() && fine < 2) {
      fine ++;
      turning = true;
    }
  }


  leftStatus = leftMotor.set_rpm(-leftSetpoint);
  rightStatus = rightMotor.set_rpm(rightSetpoint);


  #ifndef USETELOMETER
    Serial.print("lStat:"); Serial.print(leftStatus);
    Serial.print(",rStat:"); Serial.print(rightStatus);
    Serial.print(",lRPM:"); Serial.print(leftRPM);
    Serial.print(",rRPM:"); Serial.print(rightRPM);
    Serial.print(",lPos:"); Serial.print(leftPos);
    Serial.print(",rPos:"); Serial.print(rightPos);

    Serial.println();
  #endif

  #ifdef USETELOMETER
    Telemetry::sendPacket(Telemetry::loopTime);
    Telemetry::sendPacket(Telemetry::test);
    Telemetry::sendPacket(Telemetry::leftStatus);
    Telemetry::sendPacket(Telemetry::rightStatus);
    Telemetry::sendPacket(Telemetry::leftRPM);
    Telemetry::sendPacket(Telemetry::rightRPM);
    Telemetry::sendPacket(Telemetry::leftPos);
    Telemetry::sendPacket(Telemetry::rightPos);
    Telemetry::sendPacket(Telemetry::position);
    Telemetry::sendPacket(Telemetry::heading);
    Telemetry::sendPacket(Telemetry::rightSetpoint);
    Telemetry::sendPacket(Telemetry::leftSetpoint);
    Telemetry::sendPacket(Telemetry::targetPosition);
    Telemetry::sendPacket(Telemetry::aoirsetna);
    Telemetry::update();
  #endif

  test++;

  delay(20);
}