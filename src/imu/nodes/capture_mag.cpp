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
#include <imu/MagneticFieldBatch.h>

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
        int frameSize_;

        AxisData dataMag_;

        sensor_msgs::MagneticField msgMag_;
        imu::MagneticFieldBatch msgMagBatch_;
        int nbSamplesBatch_;

        int fdMag_;

        CaptureNode() : node_("~"){

        	node_.param("output", outputMag_, std::string("/imu/mag"));
        	node_.param("device", deviceMag_, std::string("/dev/lsm303d_mag"));
        	node_.param("rate", rate_, 0.0);
        	node_.param("calibrate", calibrate_, false);
        	node_.param("frame_size", frameSize_, 1);

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

			nbSamplesBatch_ = 0;
			if (frameSize_ > 1){
				msgMagBatch_.stamps.resize(frameSize_);
				msgMagBatch_.magnetic_fields.resize(frameSize_);
				msgMagBatch_.header.frame_id = "imu_link";

				pubMag_ = node_.advertise<imu::MagneticFieldBatch>(outputMag_, 10);
			}else{
				msgMag_.header.frame_id = "imu_link";

				pubMag_ = node_.advertise<sensor_msgs::MagneticField>(outputMag_, 10);
			}
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

        void applyMagneticCorrection(geometry_msgs::Vector3& magnetic_field){

			//Correction constants
			const float magRotz[3][3] = { {        1.0,        0.0, -0.07321487  } ,
										  {        0.0,        1.0, -0.00444791  } ,
										  {0.07901695 , 0.00446781,        1.0  } };

			const float magRotxy[3][3] = { { 0.76908318 ,-0.6391487 ,       0.0  } ,
										   { 0.6391487  , 0.76908318,       0.0  } ,
										   {        0.0,        0.0 ,       1.0  } };
			const float magFacxy[2] = { 1.07176589 , 0.9372419};

			const float xOffset = 0.000010176;
			const float yOffset = 0.00004176;
			const float zOffset = -0.000029264;

            float temp_mag_field_x = 0.0;
            float temp_mag_field_y = 0.0;
            float temp_mag_field_z = 0.0;

			
            float temp_mag_field_soft_x = 0.0;
            float temp_mag_field_soft_y = 0.0;
            float temp_mag_field_soft_z = 0.0;
            
            //Apply centering offsets (hard-iron correction)
			magnetic_field.x -= xOffset;
			magnetic_field.y -= yOffset;
			magnetic_field.z -= zOffset;

			
            
            //Apply bank correction
			temp_mag_field_x = magRotz[0][0]*magnetic_field.x +
							   magRotz[0][1]*magnetic_field.y +
							   magRotz[0][2]*magnetic_field.z  ;
			temp_mag_field_y = magRotz[1][0]*magnetic_field.x +
							   magRotz[1][1]*magnetic_field.y +
							   magRotz[1][2]*magnetic_field.z  ;
			temp_mag_field_z = magRotz[2][0]*magnetic_field.x +
							   magRotz[2][1]*magnetic_field.y +
							   magRotz[2][2]*magnetic_field.z  ;

			//Apply soft-iron correction
			temp_mag_field_soft_x = magRotxy[0][0]*temp_mag_field_x +
								    magRotxy[0][1]*temp_mag_field_y +
							        magRotxy[0][2]*temp_mag_field_z  ;
			temp_mag_field_soft_y = magRotxy[1][0]*temp_mag_field_x +
								    magRotxy[1][1]*temp_mag_field_y +
								    magRotxy[1][2]*temp_mag_field_z  ;
			temp_mag_field_soft_z = magRotxy[2][0]*temp_mag_field_x +
								    magRotxy[2][1]*temp_mag_field_y +
								    magRotxy[2][2]*temp_mag_field_z  ;

			temp_mag_field_soft_x *= magFacxy[0];
			temp_mag_field_soft_y *= magFacxy[1];

			magnetic_field.x = magRotxy[0][0]*temp_mag_field_soft_x +
							   magRotxy[1][0]*temp_mag_field_soft_y +
							   magRotxy[2][0]*temp_mag_field_soft_z  ;
			magnetic_field.y = magRotxy[0][1]*temp_mag_field_soft_x +
							   magRotxy[1][1]*temp_mag_field_soft_y +
							   magRotxy[2][1]*temp_mag_field_soft_z  ;
			magnetic_field.z = magRotxy[0][2]*temp_mag_field_soft_x +
							   magRotxy[1][2]*temp_mag_field_soft_y +
							   magRotxy[2][2]*temp_mag_field_soft_z  ;
		}

        bool spin() {

        	bool magDataReady;

        	ros::Rate rate(0.0);
        	if (rate_ > 0.0){
        		rate = ros::Rate(rate_);
        	}

        	bool published = false;
        	while (node_.ok()) {
                
            	magDataReady = false;
            	while (!magDataReady){
            		magDataReady = waitMag();
            	}

            	if (frameSize_ > 1){
					msgMagBatch_.stamps[nbSamplesBatch_] = ros::Time::now();

					// Convert ug to Tesla
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
					// NOTE: inverted x and y axis intentional since lsm303d and l3dg20 were not using same axis reference on IMU board.
					msgMagBatch_.magnetic_fields[nbSamplesBatch_].x = ((double) dataMag_.y)/1000000 /10000.0;
					msgMagBatch_.magnetic_fields[nbSamplesBatch_].y = -((double) dataMag_.x)/1000000 /10000.0;
					msgMagBatch_.magnetic_fields[nbSamplesBatch_].z = ((double) dataMag_.z)/1000000 /10000.0;

					if (calibrate_){
						applyMagneticCorrection(msgMagBatch_.magnetic_fields[nbSamplesBatch_]);
					}
					nbSamplesBatch_++;

					if (nbSamplesBatch_ == frameSize_){
						msgMagBatch_.header.stamp = ros::Time::now();
						pubMag_.publish(msgMagBatch_);
						nbSamplesBatch_ = 0;
						published = true;
					}
				} else{

					msgMag_.header.stamp = ros::Time::now();

					// Convert ug to Tesla
					// NOTE: using standard axis orientation, see http://www.ros.org/reps/rep-0103.html
					// NOTE: inverted x and y axis intentional since lsm303d and l3dg20 were not using same axis reference on IMU board.
					msgMag_.magnetic_field.x = ((double) dataMag_.y)/1000000 /10000.0;
					msgMag_.magnetic_field.y = -((double) dataMag_.x)/1000000 /10000.0;
					msgMag_.magnetic_field.z = ((double) dataMag_.z)/1000000 /10000.0;

					if (calibrate_){
						applyMagneticCorrection(msgMag_.magnetic_field);
					}
					pubMag_.publish(msgMag_);
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
    ros::init(argc, argv, "capture_mag");

    CaptureNode a;
    a.spin();
    return 0;
}
