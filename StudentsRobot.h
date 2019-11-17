/*
 * StudentsRobot.h
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#ifndef STUDENTSROBOT_H_
#define STUDENTSROBOT_H_
#include "config.h"
#include <Arduino.h>
#include "src/pid/ServoEncoderPIDMotor.h"
#include "src/pid/HBridgeEncoderPIDMotor.h"
#include "src/pid/ServoAnalogPIDMotor.h"
#include <ESP32Servo.h>

#include "DrivingChassis.h"
#include "src/commands/IRCamSimplePacketComsServer.h"
#include "src/commands/GetIMU.h"

/**
 * @enum RobotStateMachine
 * These are sample values for a sample state machine.
 * Feel free to add ot remove values from here
 */
enum RobotStateMachine {
	StartupRobot = 0,
	StartRunning = 1,
	Running = 2,
	Halting = 3,
	Halt = 4,
	WAIT_FOR_MOTORS_TO_FINNISH = 5,
	WAIT_FOR_TIME = 6,
	WAIT_FOR_DISTANCE = 7

};
/**
 * @enum ComStackStatusState
 * These are values for the communications stack
 * Don't add any more or change these. This is how you tell the GUI
 * what state your robot is in.
 */
enum ComStackStatusState {
	Ready_for_new_task = 0,
	Heading_to_pickup = 1,
	Waiting_for_approval_to_pickup = 2,
	Picking_up = 3,
	Heading_to_Dropoff = 4,
	Waiting_for_approval_to_dropoff = 5,
	Dropping_off = 6,
	Heading_to_safe_zone = 7,
	Fault_failed_pickup = 8,
	Fault_failed_dropoff = 9,
	Fault_excessive_load = 10,
	Fault_obstructed_path = 11,
	Fault_E_Stop_pressed = 12
};
/**
 * @class StudentsRobot
 */
class StudentsRobot {
private:
	PIDMotor * motor1;
	PIDMotor * motor2;
	PIDMotor * motor3;
	Servo * servo;
	float lsensorVal = 0;
	float rsensorVal = 0;
	long nextTime = 0;
	long startTime = 0;
	RobotStateMachine nextStatus = StartupRobot;
	IRCamSimplePacketComsServer * IRCamera;
	GetIMU * IMU;
	DrivingChassis * chassis;

	float targetDist;
	float radius = 27.4; //27.4 mm
	float track = 251.5; //227 mm
public:
	/**
	 * Constructor for StudentsRobot
	 *
	 * attach the 4 actuators
	 *
	 * these are the 4 actuators you need to use for this lab
	 * all 4 must be attached at this time
	 * DO NOT reuse pins or fail to attach any of the objects
	 *
	 */
	StudentsRobot(PIDMotor * motor1, PIDMotor * motor2, PIDMotor * motor3,
			Servo * servo, IRCamSimplePacketComsServer * IRCam, GetIMU * imu);
	/**
	 * Command status
	 *
	 * this is sent upstream to the Java GUI to notify it of current state
	 */
	ComStackStatusState myCommandsStatus = Ready_for_new_task;
	/**
	 * This is internal data representing the runtime status of the robot for use in its state machine
	 */
	RobotStateMachine status = StartupRobot;

	/**
	 * pidLoop This functoion is called to let the StudentsRobot controll the running of the PID loop functions
	 *
	 * The loop function on all motors needs to be run when this function is called and return fast
	 */
	void pidLoop();
	/**
	 * updateStateMachine use the stub state machine as a starting point.
	 *
	 * the students state machine can be updated with this function
	 */
	void updateStateMachine();
};

#endif /* STUDENTSROBOT_H_ */
