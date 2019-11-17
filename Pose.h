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
	PIDMotor * myleft;
	PIDMotor * myright;
	GetIMU * IMU;

	int xPos = 0;
	int yPos = 0;
	int heading = 0;
	double lastEncoder0, lastEncoder1, lastIMUHeading = 0;
	unsigned long lastTimestamp = -1;
	double radius = 2.61f; //2.61
	double track = 23.45f; //23.4
	double omega = 0;

public:
	double x, y, theta = 0;

	Pose(PIDMotor * left, PIDMotor * right, GetIMU * imu);
	void updatePose();
	void reset();

};

#endif /* POSE_H_ */
