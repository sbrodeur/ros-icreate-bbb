#!/usr/bin/env python

# Copyright (c) 2016, Simon Brodeur
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without 
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright 
#    notice, this list of conditions and the following disclaimer.
#   
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software without
#    specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
# IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
# NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
# OF SUCH DAMAGE.

import sys
import array
import pyaudio
import itertools
import numpy

import rospy
from std_msgs.msg import Header, MultiArrayLayout, MultiArrayDimension
from audio.msg import AudioData

class AudioCapture():
    
    FORMAT = pyaudio.paInt16
    
    def __init__(self, name, device, channels, rate, bufferSize):
        self.name = name
        self.device = device
        self.channels = channels
        self.rate = rate
        self.bufferSize = bufferSize
        self.p = pyaudio.PyAudio()
        
        # Print all devices information
        for i in range(self.p.get_device_count()):
            devName = self.p.get_device_info_by_index(i)['name'];
            devIndex = self.p.get_device_info_by_index(i)['index']
            print 'Found device: %s , index %d' % (devName, devIndex)
        
        devinfo = self.p.get_device_info_by_index(device)
        if not self.p.is_format_supported(rate,
                                      input_device=devinfo['index'],
                                      input_channels=devinfo['maxInputChannels'],
                                      input_format=pyaudio.paInt16):
            raise Error('Unsupported configuration')
        
        
        self.stream = self.p.open(input_device_index=device,
                                 format=AudioCapture.FORMAT,
                                 channels=channels,
                                 rate=rate,
                                 input=True,
                                 frames_per_buffer=self.bufferSize)
 
        # Create publisher
        output = rospy.get_param('~output', 'audio' + '/' + self.name + '/raw')
        self.pub = rospy.Publisher(output, AudioData, queue_size=20)
 
    def close(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        
    def spinOnce(self):
    	try:
            # Read a frame
            frame = self.stream.read(self.bufferSize)
            data = array.array('h', frame)
            
            # Deinterleave channel data        
            chanData = []
            for i in range(self.channels):
                chanData.append(data[i::self.channels])
            
    		# Publish channel data
    		msg = AudioData()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.fs = self.rate
            rows = self.channels
            samples = len(chanData[0])
            msg.layout = MultiArrayLayout([MultiArrayDimension('channels', rows, rows*samples),
    		                               MultiArrayDimension('samples', samples, samples)], 0)
            msg.data = list(itertools.chain.from_iterable(chanData))
            self.pub.publish(msg)
        except IOError:
            rospy.logerr('Unable to read audio stream!')

    def spin(self):
        while not rospy.is_shutdown():
            self.spinOnce()
        
if __name__ == '__main__':
    
    capture = None
    try:
        rospy.init_node('audio_capture', log_level=rospy.INFO, anonymous=True)
        device = rospy.get_param('~device', 0)
        rate = rospy.get_param('~rate', 16000)
        channels = rospy.get_param('~channels', 1)
        name = rospy.get_param('~mic_name', 'default')
        bufferSize = rospy.get_param('~buffer_size', 2048)
        
        capture = AudioCapture(name, device, channels, rate, bufferSize)
        capture.spin()
    except rospy.ROSInterruptException: pass
    
    if capture != None:
        capture.close()
