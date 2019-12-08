/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "StudentsRobot.h"

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu) {
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	IRCamera = IRCam;
	IMU = imu;
#if defined(USE_IMU)
	IMU->setXPosition(200);
	IMU->setYPosition(0);
	IMU->setZPosition(0);
#endif
	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 5; //
	motor2->myPID.sampleRateMs = 5; //
	motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.00015, 0, 0);
	motor2->myPID.setpid(0.00015, 0, 0);
	motor3->myPID.setpid(0.00015, 0, 0);

	motor1->velocityPID.setpid(0.005, 0.0001, 0);
	motor2->velocityPID.setpid(0.005, 0.00015, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					1.0 * // motor to arm stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			1400, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	// Set up the Analog sensors
	/*pinMode(ANALOG_SENSE_ONE, ANALOG);
	 pinMode(ANALOG_SENSE_TWO, ANALOG);
	 pinMode(ANALOG_SENSE_THREE, ANALOG);
	 pinMode(ANALOG_SENSE_FOUR, ANALOG);*/
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	pinMode(STEPPER_DIRECTION, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	pinMode(WII_CONTROLLER_DETECT, OUTPUT);
	chassis = new DrivingChassis(motor1, motor2, track, radius, IMU);
	pathfinder = new Pathfinder();
	UltraSonicServo.attach(ULTRASONIC_SERVO_PIN);
	IR_SERVO.attach(IR_SERVO_PIN);
	pinMode(US_TRIG_PIN, OUTPUT);
	pinMode(US_ECHO_PIN, INPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_DIRECTION, OUTPUT);
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	int distance;
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	chassis->loop();
	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");

		break;
	case StartRunning:
		Serial.println("Start Running");

		digitalWrite(H_BRIDGE_ENABLE, 1);
		// Start an interpolation of the motors
		//motor1->startInterpolationDegrees(720, 1000, SIN);
		//motor2->startInterpolationDegrees(720, 1000, SIN);
		//motor3->startInterpolationDegrees(motor3->getAngleDegrees(), 1000, SIN);

		digitalWrite(STEPPER_DIRECTION, LOW);
		motor1->overrideCurrentPosition(0);
		motor2->overrideCurrentPosition(0);
		chassis->pose->reset();
		clearAdjencies();

		//path = pathfinder->pathFindTest(0, 0, 4, 4);
		path = pathfinder->generateInitialPath();
		/*path.pop_front();
		 path.pop_front();
		 path = pathfinder->addBuildingSearch(path, path.front()->nodes[0]); */
		//pathfinder->printNodes(path);
		//pathfinder->addBuildingsAndRoadBlock();
		/*
		 motor1->setVelocityDegreesPerSecond(-150);
		 motor2->setVelocityDegreesPerSecond(333);
		 //targetDist = chassis->chassisRotationToWheelDistance(90);
		 targetDist = chassis->distanceToWheelAngle(293);
		 */
		//radius = 2.74 cm
		//arc: 670.8f, m1 = 150, m2 = -333
		//chassis->DriveStraight(200);
		//targetDist = chassis->distanceToWheelAngle(1000);
		//UltraSonicServo.write(90); //0, 180
		status = Running; // set the state machine to wait for the motors to finish
		nextStatus = Halting; // the next status to move to when the motors finish
		startTime = now + 1000; // the motors should be done in 1000 ms
		nextTime = startTime + 1000; // the next timer loop should be 1000ms after the motors stop
		break;
	case Running:
		// Set up a non-blocking 1000 ms delay
		Serial.println("running");
		//IR_SERVO.write(15); //15, 170
		//UltraSonicServo.write(0);
		//Serial.println(IR_SERVO.read());

		stepper_target = 200;
		status = WAIT_FOR_TIME;
		nextTime = nextTime + 100; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		nextStatus = StartPath; //StartPath
		//motor1->setVelocityDegreesPerSecond(30);
		//Serial.println(motor1->getAngleDegrees());
		//Serial.println(motor2->getAngleDegrees());

		//int reading = readUltrasonic();
		//Serial.println(readUltrasonic());
		//Serial.println(UltraSonicServo.read());

		// Do something
		if (!digitalRead(BOOT_FLAG_PIN)) {
			Serial.println(
					" Running State Machine " + String((now - startTime)));
#if defined(USE_IMU)
			IMU->print();
#endif
#if defined(USE_IR_CAM)
			IRCamera->print();
#endif

		}
		break;
	case StartPath:
		InitialDrive(StartPath, Pathfinding);
		break;
	case Pathfinding:
		PathfindingStateMachine(Pathfinding, Halting);
		break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()
				&& motor3->isInterpolationDone()) {
			status = nextStatus;
		}
		break;
	case WAIT_FOR_DISTANCE:
		chassis->DriveStraight(velocity, degrees);
		if (abs(motor1->getAngleDegrees()) >= targetDist) {
			status = WAIT_FOR_TIME;
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);

			motor1->overrideCurrentPosition(0);
			motor2->overrideCurrentPosition(0);

			nextTime = millis() + 200;
		}
		break;
	case WAIT_FOR_STEPPER:
		stepper_counter++;
		Serial.println(stepper_counter);
		stepped = !stepped;

		if (stepped)
			digitalWrite(STEPPER_STEP, HIGH);
		else
			digitalWrite(STEPPER_STEP, LOW);

		nextTime = millis() + 5;
		status = WAIT_FOR_TIME;
		nextStatus = WAIT_FOR_STEPPER;

		if (stepper_counter >= stepper_target) {
			stepper_counter = 0;
			status = Halting;
		}
		break;
	case WAIT_FOR_TURN:
		if (chassis->TurnBetter(degrees)) {
			status = WAIT_FOR_TIME;
			motor1->setVelocityDegreesPerSecond(0);
			motor2->setVelocityDegreesPerSecond(0);
			nextTime = millis() + 200;

			motor1->overrideCurrentPosition(0);
			motor2->overrideCurrentPosition(0);
		}
		break;
	case SCAN_LEFT:
		/*if (!path.front()->nodes[(cardinalDirection + 3) % 4]->street) {
		 adjacencies[(cardinalDirection + 3) % 4] = 1;
		 }
		 status = SCAN_MIDDLE;*/
		nextStatus = SCAN_MIDDLE;

		distance = readUltrasonic();
		Serial.println(distance);
		if (distance > 20 && distance < 35) {
			Serial.print("Building left");
			int adj = (cardinalDirection - 1) % 4;
			adjacencies[adj] = 1;
			IR_SERVO.write(15);
			nextStatus = SCAN_LEFT_IR;
		}

		UltraSonicServo.write(90);
		nextTime = millis() + 500;
		status = WAIT_FOR_TIME;
		break;
	case SCAN_MIDDLE:
		if (!path.front()->nodes[cardinalDirection]->street) {
			adjacencies[cardinalDirection] = 1;
		}

		distance = readUltrasonic();
		Serial.println(distance);
		if (distance > 20 && distance < 35) {
			Serial.print("road block middle");
			int adj = cardinalDirection;
			adjacencies[adj] = 1;
		}
		nextTime = millis() + 500;
		UltraSonicServo.write(180);
		status = WAIT_FOR_TIME;
		nextStatus = SCAN_RIGHT;
		break;
	case SCAN_RIGHT:
		/*if (!path.front()->nodes[(cardinalDirection + 1) % 4]->street) {
		 adjacencies[(cardinalDirection + 1) % 4] = 1;
		 }
		 status = nextStatus;*/
		status = Pathfinding;

		distance = readUltrasonic();
		Serial.println(distance);
		if (distance > 20 && distance < 35) {
			Serial.print("Building right");
			int adj = (cardinalDirection + 1) % 4;
			adjacencies[adj] = 1;
			IR_SERVO.write(170);
			nextStatus = SCAN_RIGHT_IR;
			status = WAIT_FOR_TIME;
			nextTime = millis() + 500;
		}

		UltraSonicServo.write(0);
		break;
	case SCAN_LEFT_IR:
		if (readIRDetector()) {
			//TODO path back home
		}
		nextTime = millis() + 500;
		UltraSonicServo.write(90);
		status = WAIT_FOR_TIME;
		nextStatus = SCAN_MIDDLE;
		break;
	case SCAN_RIGHT_IR:
		if (readIRDetector()) {
			//TODO path back home
		}
		status = Pathfinding;
		break;
	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();

		pathfinder->printNodes(visited_path);

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;

	}
	digitalWrite(WII_CONTROLLER_DETECT, 0);
}

/**
 * This is run fast and should return fast
 *
 * You call the PIDMotor's loop function. This will update the whole motor control system
 * This will read from the concoder and write to the motors and handle the hardware interface.
 */
void StudentsRobot::pidLoop() {
	motor1->loop();
	motor2->loop();
	motor3->loop();
}

void StudentsRobot::InitialDrive(RobotStateMachine currentState,
		RobotStateMachine nextState) {
	static int start_state = 0;
	switch (start_state) {
	case 0:
		targetDist = chassis->distanceToWheelAngle(430);
		status = WAIT_FOR_DISTANCE;
		nextStatus = currentState;
		start_state = 1;
		break;
	case 1:
		degrees = 90;
		cardinalDirection = 1;
		status = WAIT_FOR_TURN;
		nextStatus = currentState;
		start_state = 2;
		break;
	case 2:
		targetDist = chassis->distanceToWheelAngle(90);
		status = WAIT_FOR_DISTANCE;
		nextStatus = nextState;
		start_state = 0;
		break;
	}
}

void StudentsRobot::PathfindingStateMachine(RobotStateMachine currentState,
		RobotStateMachine nextState) {
	static int path_state = 0;
	Node* maybeRemove;
	static Node* current;
	switch (path_state) {
	case 0:
		if (!reachedDestination()) {
			current = path.front();
			path.pop_front();
			degrees += determineNextTurn(current, path.front());
			path.push_front(current);

			status = WAIT_FOR_TURN;
			nextStatus = currentState;
			path_state = 1;
		} else {
			status = nextState;
		}
		break;
	case 1:
		//TODO: Perform scanning operations & update path
		UltraSonicServo.write(0);
		status = SCAN_LEFT;
		nextStatus = currentState;
		path_state = 2;
		break;
	case 2:
		path_state = interpretAdjData();
		clearAdjencies();

		break;
	case 3:
		visited_path.push_back(path.front());
		path.pop_front();

		targetDist = chassis->distanceToWheelAngle(420);
		status = WAIT_FOR_DISTANCE;
		nextStatus = currentState;
		path_state = 0;
		maybeRemove = path.front();
		path.pop_front();
		if (maybeRemove != path.front()) {
			path.push_front(maybeRemove);
		}
		break;
	}
}

int StudentsRobot::determineNextTurn(Node* current, Node* next) {
	int turn = current->findAdj(next) * 90 - cardinalDirection * 90;
	if (turn == 270)
		turn = -90;
	else if (turn == -270)
		turn = 90;
	cardinalDirection = current->findAdj(next);
	if (turn > 0)
		chassis->IMU_offsett -= 2;
	else if (turn < 0)
		chassis->IMU_offsett += 2;

	return turn;
}

bool StudentsRobot::reachedDestination() {
	if (path.size() == 1)
		return true;
	return false;
}

int StudentsRobot::readUltrasonic() {
	long duration, distance;
	digitalWrite(US_TRIG_PIN, LOW);
	delayMicroseconds(2);
	digitalWrite(US_TRIG_PIN, HIGH);
	delayMicroseconds(10);
	digitalWrite(US_TRIG_PIN, LOW);
	duration = pulseIn(US_ECHO_PIN, HIGH);
	distance = (duration / 2) / 29.1;

	return (int) distance;
}

bool StudentsRobot::readIRDetector() {
	//TODO actually read IR Detector
	return false;
}

void StudentsRobot::clearAdjencies() {
	for (int i = 0; i < 4; i++) {
		adjacencies[i] = 0;
	}
}

int StudentsRobot::interpretAdjData() {
	for (int i = 0; i < 4; i++) {
		if (adjacencies[i] == 1) {
			Node* adj = path.front()->nodes[i];
			if (adj->street || adj->roadBlock) { //TODO remove roadblock
				Serial.println("found road block");
				adj->street = false;
				adj->roadBlock = true;

				if (i == cardinalDirection) {
					Serial.println("pathfinding around road block");

					Node* current = path.front();
					path.pop_front();
					path.pop_front();
					Node* target = path.front();

					list<Node*> newPath = pathfinder->pathfind(current, target);
					path = pathfinder->pushListBack(newPath, path);

					return 0;
				}

			} else if (adj->buildingLot) { //TODO get rid of building
				Serial.println("pathfinding around building");

				adj->buildingLot = false;
				adj->building = true;

				path = pathfinder->addBuildingSearch(path, adj);

				return 0;

			}
		}
	}
	return 3;
}
