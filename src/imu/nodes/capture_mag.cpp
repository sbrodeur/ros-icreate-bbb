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

        AxisData dataMag_;

        int fdMag_;

        CaptureNode() : node_("~"){

        	node_.param("outputMag", outputMag_, std::string("/imu/mag"));
        	node_.param("deviceMag", deviceMag_, std::string("/dev/lsm303d_mag"));
        	node_.param("rate", rate_, 15.0);

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
			printf("Reading from accelerometer:\n");
			printf("device file = %s\n", deviceMag_.c_str());
			printf("device name = %s\n", nameMag);

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
						case ABS_Y : dataMag_.y = ev.value;
						case ABS_Z : dataMag_.z = ev.value;
					}
				}else if (ev.type == EV_SYN){
					dataReady = true;
				}
			}
			return dataReady;
        }

        sensor_msgs::MagneticField applyMagneticCorrection(const sensor_msgs::MagneticField& iCaptured){
			sensor_msgs::MagneticField wNewMagField = iCaptured;

			//Correction constants
			const float magRotz[3][3] = { {        1.0,        0.0, 0.00703476  } ,
										  {        0.0,        1.0,-0.07186357  } ,
										  {-0.00698555, 0.07744466,        1.0  } };

			const float magRotxy[3][3] = { {-0.74874188, 0.66286168,        0.0  } ,
										   {-0.66286168,-0.74874188,        0.0  } ,
										   {        0.0,        0.0,        1.0  } };
			const float magFacxy[2] = { 0.93773639, 1.07111999};

			const float xOffset =  0.00002768;
			const float yOffset = -0.00000528;

			//Apply bank correction
			float flatMagFieldx = magRotz[0][0]*iCaptured.magnetic_field.x +
								  magRotz[0][1]*iCaptured.magnetic_field.y +
								  magRotz[0][2]*iCaptured.magnetic_field.z  ;
			float flatMagFieldy = magRotz[1][0]*iCaptured.magnetic_field.x +
								  magRotz[1][1]*iCaptured.magnetic_field.y +
								  magRotz[1][2]*iCaptured.magnetic_field.z  ;
			float flatMagFieldz = magRotz[2][0]*iCaptured.magnetic_field.x +
								  magRotz[2][1]*iCaptured.magnetic_field.y +
								  magRotz[2][2]*iCaptured.magnetic_field.z  ;
			//Apply soft-iron correction

			float siCorrx = magRotxy[0][0]*flatMagFieldx +
							magRotxy[0][1]*flatMagFieldy +
							magRotxy[0][2]*flatMagFieldz  ;
			float siCorry = magRotxy[1][0]*flatMagFieldx +
							magRotxy[1][1]*flatMagFieldy +
							magRotxy[1][2]*flatMagFieldz  ;
			float siCorrz = magRotxy[2][0]*flatMagFieldx +
							magRotxy[2][1]*flatMagFieldy +
							magRotxy[2][2]*flatMagFieldz  ;

			siCorrx *= magFacxy[0];
			siCorry *= magFacxy[1];

			wNewMagField.magnetic_field.x = magRotxy[0][0]*siCorrx +
											magRotxy[1][0]*siCorry +
											magRotxy[2][0]*siCorrz  ;
			wNewMagField.magnetic_field.y = magRotxy[0][1]*siCorrx +
											magRotxy[1][1]*siCorry +
											magRotxy[2][1]*siCorrz  ;
			wNewMagField.magnetic_field.z = magRotxy[0][2]*siCorrx +
											magRotxy[1][2]*siCorry +
											magRotxy[2][2]*siCorrz  ;

			//Apply centering offsets (hard-iron correction)
			wNewMagField.magnetic_field.x += xOffset;
			wNewMagField.magnetic_field.y += yOffset;

			return wNewMagField;
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

				// Y -> North X -> East, inverted values for IMU consolidator
				msgMag.magnetic_field.x = ((double) dataMag_.x)/1000000 /10000.0;
				msgMag.magnetic_field.y = ((double) dataMag_.y)/1000000 /10000.0;
				msgMag.magnetic_field.z = ((double) dataMag_.z)/1000000 /10000.0;

				msgMag = applyMagneticCorrection(msgMag);
				pubMag_.publish(msgMag);

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
