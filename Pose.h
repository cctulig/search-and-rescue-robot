/*
 * Pose.h
 *
 *  Created on: Nov 13, 2019
 *      Author: Conrad Tulig
 */

#ifndef POSE_H_
#define POSE_H_
#include "src/pid/PIDMotor.h"
#include "src/commands/GetIMU.h"
#include "config.h"
#include <math.h>

class Pose {
private:
	int xPos = 0;
	int yPos = 0;
	int heading = 0;

	PIDMotor * myleft;
	PIDMotor * myright;
	GetIMU * IMU;

	double x, y, theta = 0;
	double lastEncoder0, lastEncoder1, lastIMUHeading = 0;
	unsigned long lastTimestamp = -1;
	double radius = 2.61f; //2.61
	double track = 23.66f;
	double omega = 0;

public:
	Pose(PIDMotor * left, PIDMotor * right, GetIMU * imu);
	void updatePose();
	bool loop();

};

#endif /* POSE_H_ */
