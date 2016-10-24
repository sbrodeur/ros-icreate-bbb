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

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input bag file')
    parser.add_option("-o", "--output", dest="output", default=None,
                      help='specify the path of the output bag file')
    (options,args) = parser.parse_args(args=args)

    rosBagInPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (rosBagInPath))

    rosBagOutPath = os.path.abspath(options.output)    
    logger.info('Using output rosbag file: %s' % (rosBagOutPath))
    
    nbImagesProcessed = 0
    nbTotalMessageProcessed = 0
    with rosbag.Bag(rosBagOutPath, 'w') as outbag:
        with rosbag.Bag(rosBagInPath, 'r') as inbag:
            
            # Read all messages
            for topic, msg, timestamp in inbag.read_messages():
            
                # Rotate right camera images (lossless)
                if topic == '/video/right/compressed':
                    img = JPEGImage(blob=msg.data)
                    rotatedImg = img.flip('vertical').flip('horizontal')
                    msg.data = rotatedImg.as_blob()
                    nbImagesProcessed += 1
                    outbag.write(topic, msg, timestamp)
                # Extract Batch formatted magnetic field messages
                elif topic =='/imu/mag':
                    if hasattr(msg, 'stamps'):
                        unbatch_topic = '/imu/mag'
                        #for i in range(0,len(msg.stamps)):
                        for i in range(len(msg.stamps)):
                            newMsg = MagneticField()
                            newMsg.header.stamp = msg.stamps[i]
                            newMsg.magnetic_field.x = msg.magnetic_fields[i].x
                            newMsg.magnetic_field.y = msg.magnetic_fields[i].y
                            newMsg.magnetic_field.z = msg.magnetic_fields[i].z
                            outbag.write(unbatch_topic, newMsg, msg.stamps[i])
                # Extract Batch  formatted Imu messages
                elif topic =='/imu/data_raw':
                    if hasattr(msg, 'stamps'):
                        unbatch_topic = '/imu/data_raw'
                        for i in range(len(msg.stamps)):
                            newMsg = Imu()
                            newMsg.header.stamp = msg.stamps[i]
                            newMsg.orientation.x = msg.orientations[i].x
                            newMsg.orientation.y = msg.orientations[i].y
                            newMsg.orientation.z = msg.orientations[i].z
                            newMsg.angular_velocity.x = msg.angular_velocities[i].x
                            newMsg.angular_velocity.y = msg.angular_velocities[i].y
                            newMsg.angular_velocity.z = msg.angular_velocities[i].z
                            newMsg.linear_acceleration.x = msg.linear_accelerations[i].x
                            newMsg.linear_acceleration.y = msg.linear_accelerations[i].y
                            newMsg.linear_acceleration.z = msg.linear_accelerations[i].z
                            outbag.write(unbatch_topic, newMsg, msg.stamps[i])
                else :
                    outbag.write(topic,msg,timestamp)


                nbTotalMessageProcessed += 1
                
                if nbTotalMessageProcessed % 100 == 0:
                    logger.info('Processed %d image messages (%d total messages)' % (nbImagesProcessed, nbTotalMessageProcessed))
    
    logger.info('Processed %d image messages (%d total messages)' % (nbImagesProcessed, nbTotalMessageProcessed))
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    
