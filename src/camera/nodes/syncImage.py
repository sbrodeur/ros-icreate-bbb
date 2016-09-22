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
from jpegtran import JPEGImage

import rospy

logger = logging.getLogger(__name__)

from sensor_msgs.msg import CompressedImage, CameraInfo

import message_filters
from message_filters import TimeSynchronizer
import itertools


class ApproximateTimeSynchronizer(TimeSynchronizer):

    """
    Approximately synchronizes messages by their timestamps.
    :class:`ApproximateTimeSynchronizer` synchronizes incoming message filters by the
    timestamps contained in their messages' headers. The API is the same as TimeSynchronizer
    except for an extra `slop` parameter in the constructor that defines the delay (in seconds)
    with which messages can be synchronized. The ``allow_headerless`` option specifies whether
    to allow storing headerless messages with current ROS time instead of timestamp. You should
    avoid this as much as you can, since the delays are unpredictable.
    """

    def __init__(self, fs, queue_size, slop, allow_headerless=False):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)
        self.allow_headerless = allow_headerless

    def add(self, msg, my_queue, my_queue_index=None):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                rospy.logwarn("Cannot use message filters with non-stamped messages. "
                              "Use the 'allow_headerless' constructor option to "
                              "auto-assign ROS time to headerless messages.")
                return
            stamp = rospy.Time.now()
        else:
            stamp = msg.header.stamp

        self.lock.acquire()
        my_queue[stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if my_queue_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:my_queue_index] + \
                self.queues[my_queue_index+1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = abs(s - stamp)
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append((s, stamp_delta))
            if not topic_stamps:
                self.lock.release()
                return
            topic_stamps = sorted(topic_stamps, key=lambda x: x[1])
            stamps.append(topic_stamps)
        for vv in itertools.product(*[zip(*s)[0] for s in stamps]):
            vv = list(vv)
            # insert the new message
            if my_queue_index is not None:
                vv.insert(my_queue_index, stamp)
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
                break  # fast finish after the synchronization
        self.lock.release()

class SyncImage():
    
    def callback(self, left_image, left_info, right_image, right_info):
	    #flip image in lossless fashion
        #rospy.logwarn( "callback called")
        
        img = JPEGImage(blob=right_image.data)
        rotatedImg = img.flip('vertical')
        rotatedImg = rotatedImg.flip('horizontal')
        right_image.data = rotatedImg.as_blob()
        
        #self.stamp = (left_image.header.stamp + left_info.header.stamp + right_image.header.stamp + right_info.header.stamp)/4.0
        self.stamp = rospy.Time.now()
        left_image.header.stamp   = self.stamp
        left_info.header.stamp    = self.stamp
        right_image.header.stamp  = self.stamp
        right_info.header.stamp   = self.stamp
        
        self.left_image_pub.publish(left_image) 
        self.left_info_pub.publish(left_info)
        self.right_image_pub.publish(right_image)
        self.right_info_pub.publish(right_info)
    
    
    def __init__(self):
      
        #Subscribe to appropreate topic to flip
        syncNamespace = rospy.get_param( '~inputNamespace', '/video')
        self.left_image_sub    = message_filters.Subscriber(syncNamespace + '/left/compressed', CompressedImage)
        self.left_info_sub     = message_filters.Subscriber(syncNamespace + '/left/camera_info', CameraInfo)
        self.right_image_sub   = message_filters.Subscriber(syncNamespace + '/right/compressed', CompressedImage)
        self.right_info_sub    = message_filters.Subscriber(syncNamespace + '/right/camera_info', CameraInfo)
        
        # Create publisher
        output = rospy.get_param('~outputNamespace', '/video/sync')
        self.left_image_pub    = rospy.Publisher(output + '/left/compressed', CompressedImage, queue_size = 10)
        self.left_info_pub     = rospy.Publisher(output + '/left/camera_info', CameraInfo, queue_size = 10)
        self.right_image_pub   = rospy.Publisher(output + '/right/compressed', CompressedImage, queue_size = 10)
        self.right_info_pub    = rospy.Publisher(output + '/right/camera_info', CameraInfo, queue_size = 10)
        
        ts = ApproximateTimeSynchronizer([self.left_image_sub, self.left_info_sub, self.right_image_sub, self.right_info_sub],10,1)
        ts.registerCallback(self.callback)
    
	
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('syncImage', log_level=rospy.INFO, anonymous=True)
        
        wSyncImage = SyncImage()
        rospy.spin()

    except rospy.ROSInterruptException: pass





        
