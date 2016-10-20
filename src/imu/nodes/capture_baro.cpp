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
#include <fstream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Temperature.h>
#include <boost/lexical_cast.hpp>

using namespace std;


class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pubPres_;
        ros::Publisher pubTemp_;
        std::string outputPres_;
        std::string outputTemp_;

        std::ifstream mPresFile;
        std::ifstream mTempFile;
        int lengthT ;
        int lengthP ;
        char * currentTemperature ;
        char * currentPressure ;

        double rate_;

        CaptureNode() :
            node_("~") {

                std::string deviceLoc ;
                std::string tempProbe ;
                std::string presProbe ;

                node_.param("deviceLocation", deviceLoc, std::string("/sys/bus/i2c/drivers/bmp085/2-0077"));
                node_.param("tempProbe", tempProbe, std::string("/temp0_input"));
                node_.param("presProbe", presProbe, std::string("/pressure0_input"));
                node_.param("rateHz", rate_, 20.0);

                node_.param("outputPres", outputPres_, std::string("imu/pres/raw"));
                node_.param("outputTemp", outputTemp_, std::string("imu/temp/raw"));

                pubPres_ = node_.advertise<sensor_msgs::FluidPressure>(outputPres_, 20);
                pubTemp_ = node_.advertise<sensor_msgs::Temperature>(outputTemp_, 20);

                mTempFile.open((deviceLoc + tempProbe).c_str(), std::ifstream::in);
                mPresFile.open((deviceLoc + presProbe).c_str(), std::ifstream::in);

                lengthT = sizeof(int);
                lengthP = sizeof(double);
                currentTemperature = new char[lengthT];
                currentPressure = new char[lengthP];


            }

        virtual ~CaptureNode() {
            // Close files
            mTempFile.close();
            mPresFile.close();
            delete[] currentTemperature;
            delete[] currentPressure;
        }

        bool spin() {

                       ros::Rate rate(rate_);
            while (node_.ok()) {

                //read from files

                if(mTempFile.good())
                {
                    mTempFile.seekg(0, mTempFile.beg);
                    mTempFile.getline(currentTemperature, lengthT);
                }
                else
                {
                    ROS_WARN("Could not access Temperature file!");
                }

                if(mPresFile.good())
                {
                    mPresFile.seekg(0, mPresFile.beg);
                    mPresFile.getline(currentPressure,lengthP);
                }
                else
                {
                    ROS_WARN("Could not access Pressure file!");
                }

                
                ros::Time commonTimeStamp = ros::Time::now();

                //Publish Temperature
                sensor_msgs::Temperature msgTemp;
                msgTemp.header.stamp = commonTimeStamp;
                msgTemp.header.frame_id = "imu_link";
                msgTemp.temperature = boost::lexical_cast<float>(currentTemperature)/10.0;
                pubTemp_.publish(msgTemp);

                //Publish Pressure
                sensor_msgs::FluidPressure msgPres;
                msgPres.header.stamp = commonTimeStamp;
                msgPres.header.frame_id = "imu_link";
                msgPres.fluid_pressure = float(boost::lexical_cast<double>(currentPressure));
                pubPres_.publish(msgPres);

                rate.sleep();
            }
            
            return true;
        }

};

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture_baro");

    CaptureNode a;
    a.spin();
    return 0;
}
