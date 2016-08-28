/******************************************************************************
 * 
 * Copyright (c) 2014, Simon Brodeur
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 *  - Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *  - Neither the name of the NECOTIS research group nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <unistd.h>
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/UInt8MultiArray.h>
#include <imu/ImuPacket.h>

#include <imu/L3GD20Gyro.h>
#include <imu/LMS303.h>

using namespace std;

#define I2CBus	2

class CaptureNode {

public:
	ros::NodeHandle node_;
	ros::Publisher pub_;
	std::string output_;

	L3GD20Gyro gyro_;
	LMS303 lms303_;
	double rate_;

	CaptureNode() :
			node_("~"), gyro_(I2CBus, 0x6B),  lms303_(I2CBus, 0x1D){

		gyro_.setGyroDataRate(DR_GYRO_100HZ);
		gyro_.setGyroScale(SCALE_GYRO_245dps);

		lms303_.setMagDataRate(DR_MAG_100HZ);
		lms303_.setMagScale(SCALE_MAG_2gauss);
		lms303_.setAccelDataRate(DR_ACCEL_100HZ);
		lms303_.setAccelScale(SCALE_ACCEL_4g);

		node_.param("output", output_, std::string("imu/raw"));
		node_.param("rate", rate_, 15.0);

		pub_ = node_.advertise<imu::ImuPacket>(output_, 1);
	}

	virtual ~CaptureNode() {
		// Empty
	}

	bool spin() {
		ros::Rate rate(rate_);
		while (node_.ok()) {
			lms303_.readFullSensorState();
			gyro_.readFullSensorState();

			imu::ImuPacket msg;

			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/imu";

			msg.fs = (int) rate_;
			msg.accelX = lms303_.getAccelX();
			msg.accelY = lms303_.getAccelY();
			msg.accelZ = lms303_.getAccelZ();
			msg.pitch = lms303_.getPitch();
			msg.roll = lms303_.getRoll();
			msg.magX = lms303_.getMagX();
			msg.magY = lms303_.getMagY();
			msg.magZ = lms303_.getMagZ();
			msg.gyroX = gyro_.getGyroX();
			msg.gyroY = gyro_.getGyroY();
			msg.gyroZ = gyro_.getGyroZ();
			msg.coreTemp = lms303_.getTemperature();

			pub_.publish(msg);
			
			rate.sleep();
		}
		return true;
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "capture");

	CaptureNode a;
	a.spin();
	return 0;
}
