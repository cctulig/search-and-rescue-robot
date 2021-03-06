/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */

#include "StudentsRobot.h"

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * DO_NOT_USE, Servo * servo,
		IRCamSimplePacketComsServer * IRCam, GetIMU * imu) {
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");
	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	//this->motor3 = motor3;
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
	//motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.00015, 0, 0);
	motor2->myPID.setpid(0.00015, 0, 0);
	//motor3->myPID.setpid(0.00015, 0, 0);

	motor1->velocityPID.setpid(0.005, 0.0001, 0);
	motor2->velocityPID.setpid(0.005, 0.00015, 0);
	//motor3->velocityPID.setpid(0.1, 0, 0);
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
	/*motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
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
	 );*/
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
	//pinMode(WII_CONTROLLER_DETECT, OUTPUT);
	chassis = new DrivingChassis(motor1, motor2, track, radius, IMU);
	pathfinder = new Pathfinder();
	UltraSonicServo.attach(ULTRASONIC_SERVO_PIN);
	IR_SERVO.attach(IR_SERVO_PIN);
	pinMode(US_TRIG_PIN, OUTPUT);
	pinMode(US_ECHO_PIN, INPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	pinMode(STEPPER_DIRECTION, OUTPUT);
	pinMode(FILTER, ANALOG);

	lcd = new LiquidCrystal_I2C(0x27, 20, 4);
	lcd->begin(20, 4); // sixteen characters across - 2 lines
	lcd->backlight();
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	int distance;
	//digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	chassis->loop();
	//int r = analogRead(FILTER);
	//lcd->setCursor(0,2);
	//lcd->print(String(r).c_str());
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
		path.push_back(pathfinder->getNode(0, 0));
		priotity_locations = pathfinder->priorityQueue();

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
		IR_SERVO.write(15); //15, 170
		UltraSonicServo.write(0);
		//Serial.println(IR_SERVO.read());

		//Serial.println);
		//printStreet(pathfinder->getNode(4, 4), pathfinder->getNode(3, 4));

		stepper_target = 400;
		status = WAIT_FOR_TIME;
		nextTime = millis() + 1000; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		nextStatus = Pathfinding; //Pathfinding
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
	case SavingRobbin:
		DeployEscape(SavingRobbin, Pathfinding, robbinSide);
		break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()) {
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
		//Serial.println(stepper_counter);
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
			status = SavingRobbin;
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
		if (path.front()->nodes[(cardinalDirection + 3) % 4]->building) {
			adjacencies[(cardinalDirection + 3) % 4] = 1;
		}
		nextStatus = SCAN_MIDDLE;

		distance = readUltrasonic();
		UltraSonicServo.write(90);
		//Serial.println(distance);
		if (distance > 20 && distance < 35) {
			int adj = (cardinalDirection - 1) % 4;
			adjacencies[adj] = 1;
			IR_SERVO.write(15);
			nextStatus = SCAN_LEFT_IR;
			//Serial.println(distance);
		}

		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);
		lcd->setCursor(0, 0);
		lcd->print("LEFT  ");
		UltraSonicServo.write(90);

		nextTime = millis() + 1001;
		status = WAIT_FOR_TIME;
		break;
	case SCAN_MIDDLE:
		UltraSonicServo.write(90);
		middle_counter++;

		if (!path.front()->nodes[cardinalDirection]->street) {
			adjacencies[cardinalDirection] = 1;
		}

		distance = readUltrasonic();
		//Serial.println(middle_counter);
		UltraSonicServo.write(180);
		if (distance > 15 && distance < 35) {
			Serial.println("road block middle");
			int adj = cardinalDirection;
			adjacencies[adj] = 1;
		}
		UltraSonicServo.write(180);
		status = WAIT_FOR_TIME;
		nextStatus = SCAN_RIGHT;
		UltraSonicServo.write(180);
		lcd->setCursor(0, 0);
		lcd->print("MIDDLE");
		UltraSonicServo.write(180);
		lcd->setCursor(0, 0);
		lcd->print("MIDDLE");
		UltraSonicServo.write(180);
		UltraSonicServo.write(180);
		lcd->setCursor(0, 0);
		lcd->print("MIDDLE");
		UltraSonicServo.write(180);
		lcd->setCursor(0, 0);
		lcd->print("MIDDLE");
		UltraSonicServo.write(180);
		lcd->setCursor(0, 0);
		lcd->print("MIDDLE");
		UltraSonicServo.write(180);
		nextTime = millis() + 1201;
		break;
	case SCAN_RIGHT:
		UltraSonicServo.write(180);
		if (path.front()->nodes[(cardinalDirection + 1) % 4]->building) {
			adjacencies[(cardinalDirection + 1) % 4] = 1;
		}

		status = WAIT_FOR_TIME;
		nextStatus = Pathfinding;

		distance = readUltrasonic();
		UltraSonicServo.write(0);

		if (distance > 20 && distance < 35) {
			Serial.print("Building right");
			int adj = (cardinalDirection + 1) % 4;
			adjacencies[adj] = 1;
			IR_SERVO.write(180);
			nextStatus = SCAN_RIGHT_IR;
			status = WAIT_FOR_TIME;
		}

		UltraSonicServo.write(0);
		lcd->setCursor(0, 0);
		lcd->print("RIGHT ");
		UltraSonicServo.write(0);
		lcd->setCursor(0, 0);
		lcd->print("RIGHT ");
		UltraSonicServo.write(0);
		lcd->setCursor(0, 0);
		lcd->print("RIGHT ");
		nextTime = millis() + 1001;
		break;
	case SCAN_LEFT_IR:
		if (readIRDetector() && !foundRobbin) {
			lcd->setCursor(6, 2);
			lcd->print("FOUND BACON!");
			Node* curr = path.front();
			printStreet(curr, curr->nodes[(cardinalDirection + 3) % 4]);
			robbinSide = 1;
			status = SavingRobbin;
			foundRobbin = true;
			break;
		}
		UltraSonicServo.write(90);
		status = WAIT_FOR_TIME;
		nextStatus = SCAN_MIDDLE;
		nextTime = millis() + 1001;
		break;
	case SCAN_RIGHT_IR:
		if (readIRDetector() && !foundRobbin) {
			lcd->setCursor(6, 2);
			lcd->print("FOUND BACON!");
			Node* curr = path.front();
			printStreet(curr, curr->nodes[(cardinalDirection + 1) % 4]);
			robbinSide = -1;
			status = SavingRobbin;
			foundRobbin = true;
			break;
		}
		UltraSonicServo.write(0);
		lcd->setCursor(0, 0);
		lcd->print("RIGHT ");
		UltraSonicServo.write(0);
		lcd->setCursor(0, 0);
		lcd->print("RIGHT ");
		UltraSonicServo.write(0);

		status = Pathfinding;
		break;

	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		//motor3->stop();
		motor2->stop();
		motor1->stop();

		pathfinder->printNodes(visited_path);

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;

	}
	//digitalWrite(WII_CONTROLLER_DETECT, 0);
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
	//motor3->loop();
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

void StudentsRobot::DeployEscape(RobotStateMachine currentState,
		RobotStateMachine nextState, int dir) {
	static int escape_state = 0;
	switch (escape_state) {
	case 0:
		degrees += 90 * dir;
		targetDist = chassis->distanceToWheelAngle(90);
		status = WAIT_FOR_TURN;
		nextStatus = currentState;
		escape_state = 1;
		break;
	case 1:
		velocity = -200;
		targetDist = chassis->distanceToWheelAngle(80);
		status = WAIT_FOR_DISTANCE;
		nextStatus = currentState;
		escape_state = 2;
		break;
	case 2:
		status = WAIT_FOR_STEPPER;
		escape_state = 3;
		break;
	case 3:
		status = WAIT_FOR_TIME;
		nextStatus = currentState;
		nextTime = millis() + 30000;
		escape_state = 4;
		break;
	case 4:
		velocity = 200;
		targetDist = chassis->distanceToWheelAngle(80);
		status = WAIT_FOR_DISTANCE;
		nextStatus = currentState;
		escape_state = 5;
		break;
	case 5:
		degrees += -90 * dir;
		targetDist = chassis->distanceToWheelAngle(90);
		status = WAIT_FOR_TURN;
		nextStatus = nextState;
		escape_state = 0;

		priotity_locations.clear();
		priotity_locations.push_front(pathfinder->getNode(0, 0));
		path = pathfinder->pathfind(path.front(), pathfinder->getNode(0, 0));
		foundRobbin = true;
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
		if ((path.front() == pathfinder->getNode(0, 0) && foundRobbin)
				|| priotity_locations.size() == 0) {
			status = Halting;
		} else {
			if (!reachedDestination()) {
				current = path.front();
				priotity_locations.remove(current);
				path.pop_front();
				degrees += determineNextTurn(current, path.front());
				//printNodes(current, path.front());
				path.push_front(current);

				status = WAIT_FOR_TURN;
				nextStatus = currentState;
				path_state = 1;
				if (!addStart) {
					priotity_locations.push_back(pathfinder->getNode(0, 0));
					addStart = true;
				}

			} else {
				priotity_locations.remove(path.front());
				path = pathfinder->pathfind(path.front(),
						priotity_locations.front());
			}
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
		if (foundRobbin && !spaghetti) {
			path_state = 0;
			spaghetti = true;
			break;
		}
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
	if (analogRead(FILTER) > 500) {
		//lcd->setCursor(0, 3);
		//lcd->print(String(analogRead(FILTER)).c_str());
		return true;
	}
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
			if (adj->street || adj->roadBlock) {
				priotity_locations.remove(adj);
				Serial.println("found road block");
				adj->street = false;
				adj->roadBlock = true;

				if (i == cardinalDirection) {
					Serial.println("pathfinding around road block");

					path = pathfinder->pathfind(path.front(),
							priotity_locations.front());

					return 0;
				}

			} else if (adj->buildingLot && !foundRobbin) { //TODO get rid of building
				Serial.println("pathfinding around building");

				adj->buildingLot = false;
				adj->building = true;
				adj->hasChecked = true;

				//path = pathfinder->addBuildingSearch(path, adj);
				addPriorityNodes(pathfinder->newAddBuildingSearch(adj));
				path = pathfinder->pathfind(path.front(),
						priotity_locations.front());

				return 0;

			}
		}
	}
	return 3;
}

void StudentsRobot::addPriorityNodes(list<Node*> queue) {
	int length = queue.size();

	for (int i = 0; i < length; i++) {
		priotity_locations.push_front(queue.back());
		queue.pop_back();
	}
}

void StudentsRobot::printNodes(Node* current, Node* next) {
	lcd->setCursor(0, 0);
	lcd->print("C:(");

	lcd->setCursor(3, 0);
	lcd->print(current->xPos, 10);

	lcd->setCursor(4, 0);
	lcd->print(",");

	lcd->setCursor(5, 0);
	lcd->print(current->yPos, 10);

	lcd->setCursor(6, 0);
	lcd->print(")  N:(");

	lcd->setCursor(12, 0);
	lcd->print(next->xPos, 10);

	lcd->setCursor(13, 0);
	lcd->print(",");

	lcd->setCursor(14, 0);
	lcd->print(next->yPos, 10);

	lcd->setCursor(15, 0);
	lcd->print(")");
}

void StudentsRobot::printStreet(Node* current, Node* building) {
	String street;

	int dir = pathfinder->getAdjacentDirection(current, building);
	int x = current->xPos;
	int y = current->yPos;
	int num = 0;

	if (dir == 0 || dir == 2) {
		if (dir == 0 && x == 1)
			num = 100;
		else if (dir == 2 && x == 1)
			num = 200;
		else if (dir == 0 && x == 3)
			num = 300;
		else if (dir == 2 && x == 3)
			num = 400;
		else if (dir == 0 && x == 5)
			num = 500;
		else if (dir == 2 && x == 5)
			num = 600;

		lcd->setCursor(0, 3);
		lcd->print(String(num).c_str());
		lcd->setCursor(3, 3);

		if (y == 1)
			lcd->print(" Maple St.");
		else if (y == 3)
			lcd->print(" Beach St.");
		else if (y == 5)
			lcd->print(" Oak St.");
	}
	if (dir == 1 || dir == 3) {
		if (dir == 1 && y == 0)
			num = 200;
		else if (dir == 3 && y == 0)
			num = 100;
		else if (dir == 1 && y == 2)
			num = 400;
		else if (dir == 3 && y == 2)
			num = 300;
		else if (dir == 1 && y == 4)
			num = 600;
		else if (dir == 3 && y == 4)
			num = 500;

		lcd->setCursor(0, 3);
		lcd->print(String(num).c_str());
		lcd->setCursor(3, 3);

		if (x == 0)
			lcd->print(" First Ave.");
		else if (x == 2)
			lcd->print(" Second Ave.");
		else if (x == 4)
			lcd->print(" Third Ave.");
	}

	Serial.println(x);
	Serial.println(y);
	Serial.println(dir);

}
