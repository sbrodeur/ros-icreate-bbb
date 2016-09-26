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
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>

#include <imu/L3GD20Gyro.h>
#include <imu/LMS303.h>

using namespace std;

#define I2CBus	2
#define G_ACC   9.81
#define PI  3.14159

class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pubPos_;
        ros::Publisher pubMag_;
        ros::Publisher pubTemp_;
        std::string outputPos_;
        std::string outputMag_;
        std::string outputTemp_;

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

                node_.param("outputPos", outputPos_, std::string("imu/pos/raw"));
                node_.param("outputMag", outputMag_, std::string("imu/mag/raw"));
                node_.param("outputTemp", outputTemp_, std::string("imu/temp/raw"));
                node_.param("rate", rate_, 15.0);

                pubPos_ = node_.advertise<sensor_msgs::Imu>(outputPos_, 1);
                pubMag_ = node_.advertise<sensor_msgs::MagneticField>(outputMag_, 1);
                pubTemp_ = node_.advertise<sensor_msgs::Temperature>(outputTemp_, 1);
            }

        virtual ~CaptureNode() {
            // Empty
        }

        bool spin() {
            ros::Rate rate(rate_);
            while (node_.ok()) {
                lms303_.readFullSensorState();
                gyro_.readFullSensorState();
                
                // Positioning message
                sensor_msgs::Imu msgPos;

                msgPos.header.stamp = ros::Time::now();
                msgPos.header.frame_id = "imu_link";

                msgPos.orientation.x = 0;
                msgPos.orientation.y = 0;
                msgPos.orientation.z = 0;
                msgPos.orientation.w = 0;
                //msgPos.orientation          = quaternionFromPitchRoll(-(lms303_.getPitch()), 
                //                                                      -(lms303_.getRoll()) );    
                //msg.orientation_covariance         = ;
                
                //Convert to rad/sec
                //NOTE inverted x and y axis intentional. lms303 and Gyro 
                // were not using same axis reference
                
                msgPos.angular_velocity.y = -(gyro_.getGyroX()/180)*PI;
                msgPos.angular_velocity.x =  (gyro_.getGyroY()/180)*PI;
                msgPos.angular_velocity.z = -(gyro_.getGyroZ()/180)*PI;
                //msg.angular_velocity_covariance    = ;
                
                //Convert from g's to m/s/s
                msgPos.linear_acceleration.x = -G_ACC*lms303_.getAccelX();
                msgPos.linear_acceleration.y = -G_ACC*lms303_.getAccelY();
                msgPos.linear_acceleration.z = -G_ACC*lms303_.getAccelZ();
                //msg.linear_acceleration_covariance = ;

                pubPos_.publish(msgPos);

                // Magnetic orientation message
                // Convert Gauss to Tesla
                sensor_msgs::MagneticField msgMag;

                msgMag.header.stamp = ros::Time::now();
                msgMag.header.frame_id = "imu_link";

                float wXBias =  0.28;
                float wYBias = -0.06;
                float wXScale = 1; //5;
                float wYScale = 1; //5;
                float wZScale = 1; //2;
                
                // Y -> North X -> East, inverted values for IMU consolidator
                msgMag.magnetic_field.x = wXScale*(lms303_.getMagX() + wXBias)/10000.0;
                msgMag.magnetic_field.y = wYScale*(lms303_.getMagY() + wYBias)/10000.0;
                msgMag.magnetic_field.z = wZScale* lms303_.getMagZ()/10000.0;

                pubMag_.publish(msgMag);


                // Temperature message
                sensor_msgs::Temperature msgTemp;

                msgTemp.header.stamp = ros::Time::now();
                msgTemp.header.frame_id = "imu_link";

                msgTemp.temperature = lms303_.getTemperature();
                
                pubTemp_.publish(msgTemp);
                
                rate.sleep();
            }
            return true;
        }


        geometry_msgs::Quaternion quaternionFromPitchRoll(float iPitch, float iRoll)
        {
            
            float wPitchRad = iPitch/180 * PI;
            float wRollRad  = iRoll/180 * PI;
            float wYawRad = 0;

            float C1 = cos(wYawRad/2);
            float C2 = cos(wPitchRad/2);
            float C3 = cos(wRollRad/2);
            float S1 = sin(wYawRad/2);
            float S2 = sin(wPitchRad/2);
            float S3 = sin(wRollRad/2);
            
            geometry_msgs::Quaternion wQuart;
            //wQuart.w = sqrt(1.0 + C1*C2 + C1*C3 - S1*S2*S3 + C2*C3)/2.0;
            //wQuart.x = (C2*S3 + C1*S3 + S1*S2*C3)/(4.0*wQuart.w);  
            //wQuart.y = (S1*C2 + S1*C3 + C1*S2*S3)/(4.0*wQuart.w);
            //wQuart.z = (-S1*S3 + C1*S3*C3 + S2)/(4.0*wQuart.w);
            wQuart.w = C1*C2*C3 - S1*S2*S3;
            wQuart.x = S1*S2*C3 + C1*C2*S3;
            wQuart.y = S1*C2*C3 + C1*S2*S3;
            wQuart.z = C1*S2*C3 - S1*C2*S3;

            return wQuart;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture");

    CaptureNode a;
    a.spin();
    return 0;
}
