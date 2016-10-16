/******************************************************************************
 * 
 * Copyright (c) 2016, Simon Brodeur
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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <alsa/asoundlib.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <audio/AudioData.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using namespace std;

class CaptureNode {

    public:
        ros::NodeHandle node_;
        ros::Publisher pub_;

        std::string deviceName_;
        int rate_;
        int channels_;
        std::string micName_;
        int bufferSize_;
        std::string outputName_;

        snd_pcm_t *capture_handle_;
        snd_pcm_hw_params_t *hw_params_;
        char *buffer_;

        CaptureNode() : node_("~"){

			node_.param("device", deviceName_, std::string("default"));
			node_.param("mic_name", micName_, std::string("default"));
			node_.param("rate", rate_, 16000);
			node_.param("channels", channels_, 1);
			node_.param("buffer_size", bufferSize_, 2048);
			node_.param("output", outputName_, "/audio/" + micName_ + "/raw");

			pub_ = node_.advertise<audio::AudioData>(outputName_, 10);

			int err;
			if ((err = snd_pcm_open (&capture_handle_, deviceName_.c_str(), SND_PCM_STREAM_CAPTURE, 0)) < 0) {
				fprintf (stderr, "cannot open audio device %s (%s)\n",
						deviceName_.c_str(),
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_malloc (&hw_params_)) < 0) {
				fprintf (stderr, "cannot allocate hardware parameter structure (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_any (capture_handle_, hw_params_)) < 0) {
				fprintf (stderr, "cannot initialize hardware parameter structure (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_set_access (capture_handle_, hw_params_, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
				fprintf (stderr, "cannot set access type (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_set_format (capture_handle_, hw_params_, SND_PCM_FORMAT_S16_LE)) < 0) {
				fprintf (stderr, "cannot set sample format (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_set_rate_near (capture_handle_, hw_params_, (unsigned int*) &rate_, 0)) < 0) {
				fprintf (stderr, "cannot set sample rate (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params_set_channels (capture_handle_, hw_params_, (unsigned int) channels_)) < 0) {
				fprintf (stderr, "cannot set channel count (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			if ((err = snd_pcm_hw_params (capture_handle_, hw_params_)) < 0) {
				fprintf (stderr, "cannot set parameters (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			snd_pcm_hw_params_free (hw_params_);

			if ((err = snd_pcm_prepare (capture_handle_)) < 0) {
				fprintf (stderr, "cannot prepare audio interface for use (%s)\n",
						 snd_strerror (err));
				exit (1);
			}

			buffer_ = (char*) malloc(bufferSize_ * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * channels_);
        }

        virtual ~CaptureNode() {
			free(buffer_);
			snd_pcm_close (capture_handle_);
        }

        bool spin() {
            while (node_.ok()) {
                
            	int err;
            	if ((err = snd_pcm_readi (capture_handle_, buffer_, bufferSize_)) != bufferSize_) {
					fprintf (stderr, "read from audio interface failed (%s)\n",
						   err, snd_strerror (err));
					exit (1);
				}

                // AudioData message
                audio::AudioData msg;

                msg.header.stamp = ros::Time::now();
                msg.header.frame_id = "camera_link";
                msg.fs = rate_;

                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
				msg.layout.dim[0].size = bufferSize_;
				msg.layout.dim[0].stride = channels_*bufferSize_;
				msg.layout.dim[0].label = "frames";
				msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[1].size = channels_;
                msg.layout.dim[1].stride = channels_;
                msg.layout.dim[1].label = "channels";

				msg.data.resize(bufferSize_ * channels_);
				memcpy(&(msg.data[0]), (int16_t*) &buffer_[0], bufferSize_ * snd_pcm_format_width(SND_PCM_FORMAT_S16_LE) / 8 * channels_);
                
                pub_.publish(msg);
                
            }
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "audio_capture");

    CaptureNode a;
    a.spin();
    return 0;
}
