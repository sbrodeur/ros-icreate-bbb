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

using namespace std;

#define G_ACC   9.81
#define PI  3.14159

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

        AxisData dataAccel_;
        AxisData dataGyro_;

        int fdAccel_;
        int fdGyro_;

        CaptureNode() : node_("~"){

        	node_.param("outputPos", outputPos_, std::string("/imu/data_raw"));
        	node_.param("deviceAccel", deviceAccel_, std::string("/dev/lsm303d_acc"));
        	node_.param("deviceGyro", deviceGyro_, std::string("/dev/l3gd20_gyr"));
        	node_.param("rate", rate_, 15.0);

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

			//std::string command = "echo > " + " "
			//int system();

			/*gyro_.setGyroDataRate(DR_GYRO_800HZ);
			gyro_.setGyroScale(SCALE_GYRO_245dps);
			lms303_.setAccelDataRate(DR_ACCEL_100HZ);
			lms303_.setAccelScale(SCALE_ACCEL_4g);*/

			pubPos_ = node_.advertise<sensor_msgs::Imu>(outputPos_, 10);
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

            ros::Rate rate(rate_);
            while (node_.ok()) {
                
            	accelDataReady = false;
            	gyroDataReady = false;
            	while (!accelDataReady && !gyroDataReady){
            		if (!accelDataReady){
            			accelDataReady = waitAccel();
            		}
            		if (!gyroDataReady){
            			gyroDataReady = waitGyro();
            		}
            	}

                // Positioning message
                sensor_msgs::Imu msgPos;

                msgPos.header.stamp = ros::Time::now();
                msgPos.header.frame_id = "imu_link";

                msgPos.orientation.x = 0;
                msgPos.orientation.y = 0;
                msgPos.orientation.z = 0;
                msgPos.orientation.w = 0;
                
                //Convert to from udps to rad/sec
                //NOTE inverted x and y axis intentional. lms303 and Gyro 
                // were not using same axis reference

                msgPos.angular_velocity.y = -(((double)dataGyro_.x) / 1000000/180)*PI;
                msgPos.angular_velocity.x =  (((double)dataGyro_.y) / 1000000/180)*PI;
                msgPos.angular_velocity.z = -(((double)dataGyro_.z) / 1000000/180)*PI;
                //msg.angular_velocity_covariance    = ;
                
                //Convert from ug to m/s^2
                msgPos.linear_acceleration.x = -G_ACC*((double)dataAccel_.x) / 1000000;
                msgPos.linear_acceleration.y = -G_ACC*((double)dataAccel_.y) / 1000000;
                msgPos.linear_acceleration.z = -G_ACC*((double)dataAccel_.z) / 1000000;
                //msg.linear_acceleration_covariance = ;

                pubPos_.publish(msgPos);

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
