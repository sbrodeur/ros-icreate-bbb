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

#include <fstream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

#include <camera/capturev4l2.h>

using namespace std;

bool wSample_image_captured =false;


class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pub_;
        ros::Publisher pubCamInfo;
        boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
        VideoCapture* capture_;
        std::string camera_name_;
        std::string camera_info_url_;
        int width_;
        int height_;
        int framerate_;
        int exposure_;
        int focus_;
        int gain_;
        bool invert_image_;
        std::string output_;
        std::string video_device_;
        

        CaptureNode() :
            node_("~") {
                node_.param("camera_name", camera_name_, std::string("default"));

                node_.param("output", output_, std::string("/video/" + camera_name_ + "/compressed"));

                std::string nodeName;
                pub_ = node_.advertise<sensor_msgs::CompressedImage>(output_, 1);
                pubCamInfo = node_.advertise<sensor_msgs::CameraInfo>("/video/" + camera_name_ + "/camera_info",1);
                 
                node_.param("camera_info_url", camera_info_url_, std::string(""));
                cinfo_.reset( new  camera_info_manager::CameraInfoManager(node_, camera_name_,
                            camera_info_url_));

                node_.param("video_device", video_device_, std::string("/dev/video0"));
                node_.param("width", width_, 640);
                node_.param("height", height_, 480);
                node_.param("framerate", framerate_, 10);
                node_.param("exposure", exposure_, 1000);
                node_.param("focus", focus_, 0);
                node_.param("gain", gain_, 255);

                capture_ = new VideoCapture(video_device_, width_, height_, framerate_, exposure_, focus_, gain_, false);
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
            
            
            sensor_msgs::CameraInfo wCamInfo = cinfo_->getCameraInfo();
            wCamInfo.header.stamp = ros::Time::now();
            pubCamInfo.publish(wCamInfo);

            return true;
        }

        bool spin() {
            while (node_.ok()) {
                std::vector < uint8_t > frame = capture_->grabFrame();
                if (frame.size() > 0) {
                    ROS_DEBUG("Frame size: %d",frame.size());
                    frame = mjpeg2jpeg(frame);
                    publishFrame(frame);
                } else {
                    ROS_ERROR("Frame capture failed");
                    usleep(1000000);
                }
                ros::spinOnce();
            }

        }

        std::vector <uint8_t> mjpeg2jpeg(std::vector < uint8_t> frame )
        {

            uint8_t huffman_table[] = {0xFF,0xC4,0x01,0xA2,0x00,0x00,0x01,0x05,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x01,0x00,0x03,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x10,0x00,0x02,0x01,0x03,0x03,0x02,0x04,0x03,0x05,0x05,0x04,0x04,0x00,0x00,0x01,0x7D,0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA,0x11,0x00,0x02,0x01,0x02,0x04,0x04,0x03,0x04,0x07,0x05,0x04,0x04,0x00,0x01,0x02,0x77,0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA}; 
            
            uint8_t wHdr[2];
            wHdr[0] = frame[0];
            wHdr[1] = frame[1];
            frame.erase(frame.begin(), frame.begin()+2);
            std::vector<uint8_t> wFrameRebuilt(huffman_table, huffman_table+sizeof(huffman_table)/sizeof(uint8_t));

            if(wHdr[0] == 0xFF && wHdr[1] == 0xD8)
            { 
                //Header at start
                wFrameRebuilt.insert(wFrameRebuilt.begin(), wHdr, wHdr+2);
                //Original after Huffman table
                wFrameRebuilt.insert(wFrameRebuilt.end(), frame.begin(), frame.end());
            
                if (wSample_image_captured == false)
                {
                   ofstream test_image;
                   test_image.open ("SAMPLE_IMAGE.jpeg", ios::binary);
                   
                   ostream_iterator<uint8_t> output_iterator(test_image);
                   copy(wFrameRebuilt.begin(), wFrameRebuilt.end(), output_iterator);
                   
                   test_image.close();
                   wSample_image_captured = true;


                }

            }
            return wFrameRebuilt;  
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "capture_camera");

    CaptureNode a;
    a.spin();
    return 0;
}
