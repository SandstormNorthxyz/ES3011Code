#include <Arduino.h>
#include <Telemetry.h>


float dt = 0;
long last_update_time = 0;
int test = 0; 

vec2<float> pos;
angle dir;

void setup() {
  Serial.begin(115200);

  Telemetry::init();

  
  Telemetry::initPacket(Telemetry::loopTime, &dt);
  Telemetry::initPacket(Telemetry::test, &test);
  
}

void loop() {
  dt = (float)(micros() - last_update_time) / 1.0E6;
  last_update_time = micros(); // use microseconds because the main loop is running in 3 - 5 milliseconds
  // Serial.println("Hello");
  // delay(20);

  test++;
  
  Telemetry::sendPacket(Telemetry::loopTime);
  Telemetry::sendPacket(Telemetry::test);
  Telemetry::update();
  // Serial.print("\n");

  delay(20);
}