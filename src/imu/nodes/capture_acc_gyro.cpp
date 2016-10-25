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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <linux/input.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <imu/ImuBatch.h>

using namespace std;

#define G_ACC   9.81
#define PI  3.14159

#define SENSITIVITY_250		8750		/*	udps/LSB */
#define SENSITIVITY_500		17500		/*	udps/LSB */
#define SENSITIVITY_2000	70000		/*	udps/LSB */

struct AxisData{
	int x;
	int y;
	int z;
};

class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pubPos_;
        std::string outputPos_;
        std::string deviceAccel_;
        std::string deviceGyro_;
        double rate_;
        int frameSize_;

        AxisData dataAccel_;
        AxisData dataGyro_;

        sensor_msgs::Imu msgPos_;
        imu::ImuBatch msgPosBatch_;
        int nbSamplesBatch_;

        int fdAccel_;
        int fdGyro_;

        CaptureNode() : node_("~"){

        	node_.param("output", outputPos_, std::string("/imu/data_raw"));
        	node_.param("device_acc", deviceAccel_, std::string("/dev/lsm303d_acc"));
        	node_.param("device_gyro", deviceGyro_, std::string("/dev/l3gd20_gyr"));
        	node_.param("rate", rate_, 0.0);
        	node_.param("frame_size", frameSize_, 1);

        	// Adapted from: http://stackoverflow.com/questions/28841139/how-to-get-coordinates-of-touchscreen-rawdata-using-linux

			/* Open accelerometer device */
			fdAccel_ = open(deviceAccel_.c_str(), O_RDONLY);
			if (fdAccel_ == -1) {
				fprintf(stderr, "%s is not a valid device\n", deviceAccel_.c_str());
				exit (1);
			}

			/* Print accelerometer device name */
			char nameAccel[256] = "Unknown";
			ioctl(fdAccel_, EVIOCGNAME(sizeof(nameAccel)), nameAccel);
			printf("Reading from accelerometer:\n");
			printf("device file = %s\n", deviceAccel_.c_str());
			printf("device name = %s\n", nameAccel);

			/* Open gyroscope device */
			fdGyro_ = open(deviceGyro_.c_str(), O_RDONLY);
			if (fdGyro_ == -1) {
				fprintf(stderr, "%s is not a valid device\n", deviceGyro_.c_str());
				exit (1);
			}

			/* Print gyroscope device name */
			char nameGyro[256] = "Unknown";
			ioctl(fdGyro_, EVIOCGNAME(sizeof(nameGyro)), nameGyro);
			printf("Reading from gyroscope:\n");
			printf("device file = %s\n", deviceGyro_.c_str());
			printf("device name = %s\n", nameGyro);

			/*gyro_.setGyroDataRate(DR_GYRO_800HZ);
			gyro_.setGyroScale(SCALE_GYRO_245dps);
			lms303_.setAccelDataRate(DR_ACCEL_100HZ);
			lms303_.setAccelScale(SCALE_ACCEL_4g);*/

			nbSamplesBatch_ = 0;
			if (frameSize_ > 1){
				msgPosBatch_.stamps.resize(frameSize_);
				msgPosBatch_.angular_velocities.resize(frameSize_);
				msgPosBatch_.linear_accelerations.resize(frameSize_);
				msgPosBatch_.orientations.resize(frameSize_);

				msgPosBatch_.header.frame_id = "imu_link";
				for (unsigned int i=0; i<frameSize_; i++){
					msgPosBatch_.orientations[i].x = 0;
					msgPosBatch_.orientations[i].y = 0;
					msgPosBatch_.orientations[i].z = 0;
					msgPosBatch_.orientations[i].w = 0;
				}

				pubPos_ = node_.advertise<imu::ImuBatch>(outputPos_, 10);
			}else{
				msgPos_.header.frame_id = "imu_link";
                msgPos_.orientation.x = 0;
                msgPos_.orientation.y = 0;
                msgPos_.orientation.z = 0;
                msgPos_.orientation.w = 0;

				pubPos_ = node_.advertise<sensor_msgs::Imu>(outputPos_, 10);
			}
        }

        virtual ~CaptureNode() {
        	close(fdAccel_);
        	close(fdGyro_);
        }

        bool waitAccel(){
        	struct input_event ev;
        	const size_t ev_size = sizeof(struct input_event);
			ssize_t size;

			bool dataReady = false;
			size = read(fdAccel_, &ev, ev_size);
			if (size < ev_size) {
				fprintf(stderr, "Error size when reading\n");
			}else{
				if (ev.type == EV_ABS && (ev.code == ABS_X || ev.code == ABS_Y || ev.code == ABS_Z)) {

					// ev.value are in ug
					switch(ev.code) {
						case ABS_X : dataAccel_.x = ev.value;
							break;
						case ABS_Y : dataAccel_.y = ev.value;
							break;
						case ABS_Z : dataAccel_.z = ev.value;
							break;
					}
				}else if (ev.type == EV_SYN){
					dataReady = true;
				}
			}
			return dataReady;
        }

        bool waitGyro(){
			struct input_event ev;
			const size_t ev_size = sizeof(struct input_event);
			ssize_t size;

			bool dataReady = false;
			size = read(fdGyro_, &ev, ev_size);
			if (size < ev_size) {
				fprintf(stderr, "Error size when reading\n");
			}else{
				if (ev.type == EV_ABS && (ev.code == ABS_X || ev.code == ABS_Y || ev.code == ABS_Z)) {

					// ev.value are in udps
					switch(ev.code) {
						case ABS_X : dataGyro_.x = ev.value;
							break;
						case ABS_Y : dataGyro_.y = ev.value;
							break;
						case ABS_Z : dataGyro_.z = ev.value;
							break;
					}
				}else if (ev.type == EV_SYN){
					dataReady = true;
				}
			}
			return dataReady;
		}

        bool spin() {

        	bool accelDataReady;
        	bool gyroDataReady;

        	ros::Rate rate(0.0);
			if (rate_ > 0.0){
				rate = ros::Rate(rate_);
			}

			bool published = false;
            while (node_.ok()) {
                
            	accelDataReady = false;
            	gyroDataReady = false;
            	while (!accelDataReady || !gyroDataReady){
            		if (!accelDataReady){
            			accelDataReady = waitAccel();
            		}
            		if (!gyroDataReady){
            			gyroDataReady = waitGyro();
            		}
            	}

            	if (frameSize_ > 1){

            		msgPosBatch_.stamps[nbSamplesBatch_] = ros::Time::now();

            		// Convert to from udps to rad/sec
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
            		msgPosBatch_.angular_velocities[nbSamplesBatch_].x =  (((double)dataGyro_.x) * SENSITIVITY_250/1000000.0 * PI/180.0);
            		msgPosBatch_.angular_velocities[nbSamplesBatch_].y =  (((double)dataGyro_.y) * SENSITIVITY_250/1000000.0 * PI/180.0);
            		msgPosBatch_.angular_velocities[nbSamplesBatch_].z =  (((double)dataGyro_.z) * SENSITIVITY_250/1000000.0 * PI/180.0);

					// Convert from ug to m/s^2
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
					// NOTE: the accelerometer measures the inertial force, which is the negative of the acceleration force.
					//	     Because the imu madgwick filter expects inertial forces, we don't apply this negation.
					// NOTE: inverted x and y axis intentional since lsm303d and l3dg20 were not using same axis reference on IMU board.
            		msgPosBatch_.linear_accelerations[nbSamplesBatch_].x = G_ACC*((double)dataAccel_.y) / 1000000;
            		msgPosBatch_.linear_accelerations[nbSamplesBatch_].y =  -G_ACC*((double)dataAccel_.x) / 1000000;
            		msgPosBatch_.linear_accelerations[nbSamplesBatch_].z = G_ACC*((double)dataAccel_.z) / 1000000;

            		nbSamplesBatch_++;

            		if (nbSamplesBatch_ == frameSize_){
            			msgPosBatch_.header.stamp = ros::Time::now();
            			pubPos_.publish(msgPosBatch_);
            			nbSamplesBatch_ = 0;
            			published = true;
            		}
            	} else{
            		// Positioning message
					msgPos_.header.stamp = ros::Time::now();

					// Convert to from udps to rad/sec
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
					msgPos_.angular_velocity.x =  (((double)dataGyro_.x) * SENSITIVITY_250/1000000.0 * PI/180.0);
					msgPos_.angular_velocity.y =  (((double)dataGyro_.y) * SENSITIVITY_250/1000000.0 * PI/180.0);
					msgPos_.angular_velocity.z =  (((double)dataGyro_.z) * SENSITIVITY_250/1000000.0 * PI/180.0);

					// Convert from ug to m/s^2
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
					// NOTE: the accelerometer measures the inertial force, which is the negative of the acceleration force.
					//	     Because the imu madgwick filter expects inertial forces, we don't apply this negation.
					// NOTE: inverted x and y axis intentional since lsm303d and l3dg20 were not using same axis reference on IMU board.
					msgPos_.linear_acceleration.x = G_ACC*((double)dataAccel_.y) / 1000000;
					msgPos_.linear_acceleration.y =  -G_ACC*((double)dataAccel_.x) / 1000000;
					msgPos_.linear_acceleration.z = G_ACC*((double)dataAccel_.z) / 1000000;

					pubPos_.publish(msgPos_);
					published = true;
            	}

            	if (rate_ > 0.0 && published){
            		rate.sleep();
            		published = false;
            	}
            }
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture_acc_gyro");

    CaptureNode a;
    a.spin();
    return 0;
}
