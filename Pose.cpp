/*
 * Pose.cpp
 *
 *  Created on: Nov 13, 2019
 *      Author: Conrad Tulig
 */

#include "Pose.h"

Pose::Pose(PIDMotor * left, PIDMotor * right, GetIMU * imu) {
	myleft = left;
	myright = right;
	IMU = imu;
}

void Pose::updatePose() {
	unsigned long timestamp = millis();
	double encoder0 = myleft->getAngleDegrees();
	double encoder1 = myright->getAngleDegrees();
	//double IMUheading = ;

	/* Serial.print(encoder0);
	 Serial.print(", ");
	 Serial.println(encoder1);*/

	double deltaTime = timestamp - lastTimestamp;
	double deltaEncoder0 = encoder0 - lastEncoder0;
	double deltaEncoder1 = encoder1 - lastEncoder1;
	//double deltaIMU = IMUheading - lastIMUHeading;

	 Serial.print((deltaEncoder0/deltaTime*.04553));
	 Serial.print(", ");
	 Serial.println(deltaEncoder1/deltaTime);

	double velRight = (deltaEncoder1 / deltaTime) * 2 * 3.14 * radius / 360;
	double velLeft = deltaEncoder0 / deltaTime * 2 * 3.14 * radius / 360;
	double velAvg = (velRight + velLeft) / 2;
	double omega = (-velRight + velLeft) / track;
	double prevTheta = theta;
	theta += omega * deltaTime;

	//Serial.println(omega);

	Serial.print("velRight=");
	Serial.print(velRight);
	Serial.print(", velLeft=");
	Serial.println(velLeft);
	//Serial.print(", theta=");
	//Serial.println((theta * 360 / 2 / 3.14));*/

	/*Serial.print("cos=");
	 Serial.print(cos(theta));
	 Serial.print(", sin=");
	 Serial.print(sin(theta));*/

	x += velAvg * deltaTime * (cos(theta) + cos(prevTheta)) / 2;
	y += velAvg * deltaTime * (sin(theta) + sin(prevTheta)) / 2;

	lastEncoder0 = encoder0;
	lastEncoder1 = encoder1;
	lastTimestamp = timestamp;
	//lastIMUHeading = IMUheading;

	/*Serial.print("Final pose x=");
	 Serial.print(x);
	 Serial.print(" y=");
	 Serial.print(y);
	 Serial.print(" theta=");
	 Serial.println((theta * 360 / 2 / 3.14));*/
}

bool Pose::loop() {
	if (millis() > lastTimestamp + 20) {
		updatePose();
	}
	return false;
}
