/**@file template.ino */
#include <ESP32Servo.h>
#include <ESP32Encoder.h>
#include <Preferences.h>
#include <WiFi.h>
#include <SimplePacketComs.h>
#include <Esp32SimplePacketComs.h>
#include <Esp32WifiManager.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <DFRobotIRPosition.h>

#include "src/RobotControlCenter.h"
#include "config.h"

RobotControlCenter * controlCenter;
void setup() {
	controlCenter = new RobotControlCenter(new String(TEAM_NAME));
}

void loop() {
	controlCenter->loop(); // run the state machine pulse
}
