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

import os
import logging
import numpy as np
from optparse import OptionParser
from jpegtran import JPEGImage

from imu.msg import MagneticFieldBatch , ImuBatch
from sensor_msgs.msg import MagneticField, Imu

import rospy
import rosbag

logger = logging.getLogger(__name__)

def unbatchImu(msg):
    nbFrames = len(msg.stamps)
    for i in range(nbFrames):
        m = Imu()
        m.header.seq = nbFrames * msg.header.seq + i
        m.header.frame_id = msg.header.frame_id
        m.header.stamp = msg.stamps[i]
        m.orientation.x = msg.orientations[i].x
        m.orientation.y = msg.orientations[i].y
        m.orientation.z = msg.orientations[i].z
        m.angular_velocity.x = msg.angular_velocities[i].x
        m.angular_velocity.y = msg.angular_velocities[i].y
        m.angular_velocity.z = msg.angular_velocities[i].z
        m.linear_acceleration.x = msg.linear_accelerations[i].x
        m.linear_acceleration.y = msg.linear_accelerations[i].y
        m.linear_acceleration.z = msg.linear_accelerations[i].z
        yield m

def unbatchMagneticField(msg):
    nbFrames = len(msg.stamps)
    for i in range(nbFrames):
        m = MagneticField()
        m.header.seq = nbFrames * msg.header.seq + i
        m.header.frame_id = msg.header.frame_id
        m.header.stamp = msg.stamps[i]
        m.magnetic_field.x = msg.magnetic_fields[i].x
        m.magnetic_field.y = msg.magnetic_fields[i].y
        m.magnetic_field.z = msg.magnetic_fields[i].z
        yield m

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input bag file')
    parser.add_option("-o", "--output", dest="output", default=None,
                      help='specify the path of the output bag file')
    parser.add_option("-r", "--do-not-rotate-right-camera", action="store_true", dest="doNotRotateRightCamera", default=True,
                      help='specify to rotate right camera image stream')
    (options,args) = parser.parse_args(args=args)

    rosBagInPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (rosBagInPath))

    rosBagOutPath = os.path.abspath(options.output)    
    logger.info('Using output rosbag file: %s' % (rosBagOutPath))
    
    if options.doNotRotateRightCamera:
        logger.info('Image stream from right camera will not be rotated (vertically and horizontally)')
    
    nbTotalMessageProcessed = 0
    with rosbag.Bag(rosBagOutPath, 'w') as outbag:
        with rosbag.Bag(rosBagInPath, 'r') as inbag:
            
            # Read all messages
            for topic, msg, timestamp in inbag.read_messages():
            
                # Replace rosbag timestamps with the ones from the headers, if available
                if msg._has_header:
                    timestamp = msg.header.stamp
                else:
                    logger.warn('Topic %s provides messages than have no header: using timestamps from rosbag' % (topic))
            
                # Rename topic of imu barometer sensor
                # NOTE: this is to properly handle the old topic name
                if topic == '/imu/baro':
                    topic = '/imu/pressure'
            
                # Rotate right camera images (lossless)
                if topic == '/video/right/compressed' and not options.doNotRotateRightCamera:
                    img = JPEGImage(blob=msg.data)
                    rotatedImg = img.flip('vertical').flip('horizontal')
                    msg.data = rotatedImg.as_blob()
                    outbag.write(topic, msg, timestamp)
                    nbTotalMessageProcessed += 1
                    
                elif msg.__class__.__name__.endswith('ImuBatch'):
                    # Unbatch messages of type ImuBatch into individual Imu messages.
                    # Use the timestamps from the individual messages.
                    for m in unbatchImu(msg):
                        outbag.write(topic, m, m.header.stamp)
                        nbTotalMessageProcessed += 1
                    
                elif msg.__class__.__name__.endswith('MagneticFieldBatch'):
                    # Unbatch messages of type MagneticFieldBatch into individual MagneticField messages.
                    # Use the timestamps from the individual messages.
                    for m in unbatchMagneticField(msg):
                        outbag.write(topic, m, m.header.stamp)
                        nbTotalMessageProcessed += 1
                        
                else:
                    outbag.write(topic, msg, timestamp)
                    nbTotalMessageProcessed += 1
                            
                if nbTotalMessageProcessed % 100 == 0:
                    logger.info('Processed %d messages' % (nbTotalMessageProcessed))
                    
    logger.info('Processed %d messages' % (nbTotalMessageProcessed))
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    
