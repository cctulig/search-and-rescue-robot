/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DrivingChassis.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return distance / (2 * pi * mywheelRadiusMM) * 360;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return angle / 360 * (2 * mywheelTrackMM * pi);
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 * @param imu The object that is used to access the IMU data
 *
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM, GetIMU * imu) {
	myleft = left;
	myright = right;
	mywheelTrackMM = wheelTrackMM;
	mywheelRadiusMM = wheelRadiusMM;
	pose = new Pose(myleft, myright, imu);
	IMU = imu;
}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {
}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::turnDegrees(float degreesToRotateBase, int msDuration) {

}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 * @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	return false;
}
/**
 * loop()
 *
 * a fast loop function that will update states of the motors based on the information from the
 * imu.
 */
bool DrivingChassis::loop() {
	if (millis() > lastTimestamp + 20) {
		if (!set_offset) {
			if (cycle > 5) {
				IMU_offsett = IMU->getEULER_azimuth();
				set_offset = true;
			} else {
				cycle++;
			}
		}
		pose->updatePose();
		lastTimestamp = millis();

	}
	return false;
}

void DrivingChassis::DriveStraight(int targetVel, float degrees) {
	target_heading = degrees;
	float IMU_heading = IMU->getEULER_azimuth() - IMU_offsett;
	float curr_heading = pose->theta * 360 / 2 / 3.14;
	Serial.println(IMU_offsett);
	Serial.println(IMU->getEULER_azimuth());

	float err = target_heading - curr_heading;
	float IMU_err = target_heading - IMU_heading;

	float err_total = -(.9f * IMU_err + .1f * err);
	float velAdj = IMU_err;
	//Serial.println(err_total);

	if (velAdj > 60)
		velAdj = 60;
	if (velAdj < -60)
		velAdj = -60;

	myleft->setVelocityDegreesPerSecond(-(targetVel - velAdj));
	myright->setVelocityDegreesPerSecond(targetVel + velAdj);
}

bool DrivingChassis::Turn(float degrees) {
	target_heading = degrees;
	int direction = 1;
	float IMU_heading = IMU->getEULER_azimuth() - IMU_offsett;
	float curr_heading = pose->theta * 360 / 2 / 3.14;
	//Serial.println(IMU_offsett);
	//Serial.println(IMU->getEULER_azimuth());

	float err = target_heading - curr_heading;
	float IMU_err = target_heading - IMU_heading;

	float err_total = -(.9f * IMU_err + .1f * err);
	float velAdj = IMU_err;
	Serial.println(IMU_err);

	if (velAdj > 50)
		velAdj = 50;
	if (velAdj < -50)
		velAdj = -50;

	if (IMU_err < 4 && IMU_err > -4) {
		return true;
	}

	if (IMU_err > 0) {
		direction = 1;
	} else {
		direction = -1;
	}

	myleft->setVelocityDegreesPerSecond((150) * direction);
	Serial.println(myleft->getVelocityDegreesPerSecond());
	myright->setVelocityDegreesPerSecond((150) * direction);
	return false;
}

bool DrivingChassis::TurnBetter(float degrees) {
	target_heading = degrees;


	int IMU_heading =  (int)(IMU->getEULER_azimuth() - IMU_offsett) % 360;

	float IMU_err = target_heading - IMU_heading;

	if (IMU_err < 2 && IMU_err > -2) {
		return true;
	}
	Serial.println(target_heading);
	Serial.println(IMU_heading);

	int direction = 1;
	if (target_heading < IMU_heading) {
		direction = 1;
	} else {
		direction = -1;
	}

	myleft->setVelocityDegreesPerSecond(150 * direction);
	myright->setVelocityDegreesPerSecond(150 * direction);
	return false;
}
