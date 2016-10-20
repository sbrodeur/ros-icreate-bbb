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
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/MagneticField.h>

using namespace std;

struct AxisData{
	int x;
	int y;
	int z;
};

class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pubMag_;
        std::string outputMag_;
        std::string deviceMag_;
        double rate_;
        bool calibrate_;

        AxisData dataMag_;

        int fdMag_;

        CaptureNode() : node_("~"){

        	node_.param("outputMag", outputMag_, std::string("/imu/mag"));
        	node_.param("deviceMag", deviceMag_, std::string("/dev/lsm303d_mag"));
        	node_.param("rate", rate_, 15.0);
        	node_.param("calibrate", calibrate_, false);


        	// Adapted from: http://stackoverflow.com/questions/28841139/how-to-get-coordinates-of-touchscreen-rawdata-using-linux

			/* Open magnetometer device */
			fdMag_ = open(deviceMag_.c_str(), O_RDONLY);
			if (fdMag_ == -1) {
				fprintf(stderr, "%s is not a valid device\n", deviceMag_.c_str());
				exit (1);
			}

			/* Print magnetometer device name */
			char nameMag[256] = "Unknown";
			ioctl(fdMag_, EVIOCGNAME(sizeof(nameMag)), nameMag);
			printf("Reading from magnetometer:\n");
			printf("device file = %s\n", deviceMag_.c_str());
			printf("device name = %s\n", nameMag);

			if (calibrate_){
				printf("calibration = true\n");
			}else{
				printf("calibration = false\n");
			}

			/*
			    lms303_.setMagDataRate(DR_MAG_100HZ);
                lms303_.setMagScale(SCALE_MAG_2gauss);
			 */

			pubMag_ = node_.advertise<sensor_msgs::MagneticField>(outputMag_, 10);
        }

        virtual ~CaptureNode() {
        	close(fdMag_);
        }

        bool waitMag(){
        	struct input_event ev;
        	const size_t ev_size = sizeof(struct input_event);
			ssize_t size;

			bool dataReady = false;
			size = read(fdMag_, &ev, ev_size);
			if (size < ev_size) {
				fprintf(stderr, "Error size when reading\n");
			}else{
				if (ev.type == EV_ABS && (ev.code == ABS_X || ev.code == ABS_Y || ev.code == ABS_Z)) {

					// ev.value are in uG
					switch(ev.code) {
						case ABS_X : dataMag_.x = ev.value;
							break;
						case ABS_Y : dataMag_.y = ev.value;
							break;
						case ABS_Z : dataMag_.z = ev.value;
							break;
					}
				}else if (ev.type == EV_SYN){
					dataReady = true;
				}
			}
			return dataReady;
        }

        void applyMagneticCorrection(sensor_msgs::MagneticField& msg){

			//Correction constants
			const float magRotz[3][3] = { {        1.0,        0.0, -0.06722939  } ,
										  {        0.0,        1.0,-0.00410318  } ,
										  {0.07208762, 0.00412011,        1.0  } };

			const float magRotxy[3][3] = { {-0.716796,-0.69728293,        0.0  } ,
										   {0.69728293,-0.716796,        0.0  } ,
										   {        0.0,        0.0,        1.0  } };
			const float magFacxy[2] = { 0.93515957, 1.07450191};

			const float xOffset = 0.000003628;
			const float yOffset = 0.00003196;
			const float zOffset = -0.000032072;

			//Apply centering offsets (hard-iron correction)
			msg.magnetic_field.x -= xOffset;
			msg.magnetic_field.y -= yOffset;
			msg.magnetic_field.z -= zOffset;

			//Apply bank correction
			msg.magnetic_field.x = magRotz[0][0]*msg.magnetic_field.x +
								   magRotz[0][1]*msg.magnetic_field.y +
								   magRotz[0][2]*msg.magnetic_field.z  ;
			msg.magnetic_field.y = magRotz[1][0]*msg.magnetic_field.x +
								   magRotz[1][1]*msg.magnetic_field.y +
								   magRotz[1][2]*msg.magnetic_field.z  ;
			msg.magnetic_field.z = magRotz[2][0]*msg.magnetic_field.x +
								   magRotz[2][1]*msg.magnetic_field.y +
								   magRotz[2][2]*msg.magnetic_field.z  ;

//			//Apply soft-iron correction
//			msg.magnetic_field.x = magRotxy[0][0]*msg.magnetic_field.x +
//								   magRotxy[0][1]*msg.magnetic_field.y +
//							       magRotxy[0][2]*msg.magnetic_field.z  ;
//			msg.magnetic_field.y = magRotxy[1][0]*msg.magnetic_field.x +
//								   magRotxy[1][1]*msg.magnetic_field.y +
//								   magRotxy[1][2]*msg.magnetic_field.z  ;
//			msg.magnetic_field.z = magRotxy[2][0]*msg.magnetic_field.x +
//								   magRotxy[2][1]*msg.magnetic_field.y +
//								   magRotxy[2][2]*msg.magnetic_field.z  ;
//
//			msg.magnetic_field.x *= magFacxy[0];
//			msg.magnetic_field.y *= magFacxy[1];
//
//			msg.magnetic_field.x = magRotxy[0][0]*msg.magnetic_field.x +
//								   magRotxy[1][0]*msg.magnetic_field.y +
//								   magRotxy[2][0]*msg.magnetic_field.z  ;
//			msg.magnetic_field.y = magRotxy[0][1]*msg.magnetic_field.x +
//								   magRotxy[1][1]*msg.magnetic_field.y +
//								   magRotxy[2][1]*msg.magnetic_field.z  ;
//			msg.magnetic_field.z = magRotxy[0][2]*msg.magnetic_field.x +
//								   magRotxy[1][2]*msg.magnetic_field.y +
//								   magRotxy[2][2]*msg.magnetic_field.z  ;
		}

        bool spin() {

        	bool magDataReady;

            ros::Rate rate(rate_);
            while (node_.ok()) {
                
            	magDataReady = false;
            	while (!magDataReady){
            		magDataReady = waitMag();
            	}

            	// Magnetic orientation message
				// Convert ug to Tesla
				sensor_msgs::MagneticField msgMag;

				msgMag.header.stamp = ros::Time::now();
				msgMag.header.frame_id = "imu_link";

				// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
				// NOTE: inverted x and y axis intentional since lsm303d and l3dg20 were not using same axis reference on IMU board.
				msgMag.magnetic_field.x = ((double) dataMag_.y)/1000000 /10000.0;
				msgMag.magnetic_field.y = -((double) dataMag_.x)/1000000 /10000.0;
				msgMag.magnetic_field.z = ((double) dataMag_.z)/1000000 /10000.0;

				if (calibrate_){
					applyMagneticCorrection(msgMag);
				}
				pubMag_.publish(msgMag);

                rate.sleep();
            }
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture_mag");

    CaptureNode a;
    a.spin();
    return 0;
}
