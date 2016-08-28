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
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CompressedImage.h>
#include <camera_info_manager/camera_info_manager.h>

#include <camera/capturev4l2.h>

using namespace std;

class CaptureNode {

public:
	ros::NodeHandle node_;
	ros::Publisher pub_;
	boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
	VideoCapture* capture_;
	std::string camera_name_;
	std::string camera_info_url_;
	int width_;
	int height_;
	int framerate_;
	std::string output_;
	std::string video_device_;

	CaptureNode() :
			node_("~") {
		node_.param("camera_name", camera_name_, std::string("default"));

		node_.param("output", output_, std::string(camera_name_ + "/raw"));

		std::string nodeName;
		pub_ = node_.advertise<sensor_msgs::CompressedImage>(output_, 1);

		node_.param("camera_info_url", camera_info_url_, std::string(""));
		cinfo_.reset(
				new camera_info_manager::CameraInfoManager(node_, camera_name_,
						camera_info_url_));

		node_.param("video_device", video_device_, std::string("/dev/video0"));
		node_.param("width", width_, 640);
		node_.param("height", height_, 480);
		node_.param("framerate", framerate_, 5);

		capture_ = new VideoCapture(video_device_, width_, height_, framerate_, false);
	}

	virtual ~CaptureNode() {
		delete capture_;
	}

	bool publishFrame(const std::vector<uint8_t>& frame) {

		sensor_msgs::CompressedImage msg;

		msg.header.stamp = ros::Time::now();
		msg.format = "jpeg";
		msg.data.resize(frame.size());
		memcpy(&(msg.data[0]), &frame[0], frame.size());

		pub_.publish(msg);

		return true;
	}

	bool spin() {
		while (node_.ok()) {
			std::vector < uint8_t > frame = capture_->grabFrame();
			if (frame.size() > 0) {
				ROS_DEBUG("Frame size: %d",frame.size());
				publishFrame(frame);
			} else {
				ROS_ERROR("Frame capture failed");
				usleep(1000000);
			}
			ros::spinOnce();
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
