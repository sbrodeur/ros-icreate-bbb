#!/usr/bin/env python

# Copyright (c) 2016, Simon Carrier, Simon Brodeur
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
import os
import logging
import numpy as np

import matplotlib
import matplotlib.pyplot as plt

import rosbag

logger = logging.getLogger(__name__)

def getAllTopicsMetadata(filename, ignoredTopics, useRosbagTime=False):

    logger.debug('Reading timestamp metadata from rosbag topics')

    topicTimestamps = dict()
    topicSequenceIds = dict()
    with rosbag.Bag(filename) as bag:
        # Grab list of topics
        topics = bag.get_type_and_topic_info()[1].keys()

        # Generate Topic statisitics objects
        for topic in topics:
            if not topic in ignoredTopics:
                topicTimestamps[topic] = []
                topicSequenceIds[topic] = []

        # Process all messages
        for topic, msg, t in bag.read_messages(topics=topicTimestamps.keys()):
            if useRosbagTime:
                t = t.to_sec()
            else:
                # Get timestamp from header
                t = float(msg.header.stamp.to_sec())
            topicTimestamps[topic].append(t)

            id = msg.header.seq
            topicSequenceIds[topic].append(id)

    # Convert to sorted numpy arrays
    # NOTE: sorting is important when using capture time, since they are not guaranteed
    #       to be ordered as in the rosbag.
    logger.debug('Sorting timestamp metadata per topic')
    for topic, timestamps in topicTimestamps.iteritems():
        topicTimestamps[topic] = np.sort(np.array(timestamps))
    for topic, ids in topicSequenceIds.iteritems():
        topicSequenceIds[topic] = np.sort(np.array(ids))
    return topicTimestamps, topicSequenceIds

def plotDropsTimeDistribution(drops, centerPos, cropWindow=600, windowSizeConv=10.0):

    t = np.linspace(0.0, windowSizeConv * len(drops), len(drops))
    fig = plt.figure(figsize=(8,6), facecolor='white')

    plt.title("Total dropped messages over time")
    plt.xlabel('Time [sec]')
    plt.ylabel('Messages dropped (all topics)')
    plt.plot(t, drops, color='k')
    plt.xlim([np.min(t), np.max(t)])
    plt.axvline(centerPos, color='b')
    plt.axvline((centerPos + cropWindow/2.0), color='r')
    plt.axvline((centerPos - cropWindow/2.0), color='r')
    return fig

def getDropsTimeDistribution(topicTimestamps, dropThreshold=1.0, windowWidth=10.0, ignoreBorder=0.0):

    firstMessageStamp = -1
    lastMessageStamp = -1
    for topic, timestamps in topicTimestamps.iteritems():
        if (firstMessageStamp > timestamps[0]) or (firstMessageStamp == -1):
            firstMessageStamp = timestamps[0]
        if (lastMessageStamp < timestamps[-1]) or (lastMessageStamp == -1):
            lastMessageStamp = timestamps[-1]

    #Ignore the first ignoreBorder and last ignoreBorder seconds
    firstMessageStamp += ignoreBorder
    lastMessageStamp -= ignoreBorder

    #Timestamps bins
    timeSlices = np.arange(firstMessageStamp, lastMessageStamp, windowWidth)

    totalDropOverTime = np.zeros(np.size(timeSlices))
    for topic, timestamps in topicTimestamps.iteritems():
        for i in range(np.size(timeSlices)-1):
            timestampsWindow = timestamps[(timeSlices[i] < timestamps) & (timestamps < timeSlices[i+1])]
            
            if len(timestampsWindow) > 1:
                # Calculate relative times between messages
                timeDiffs = timestampsWindow[1:] - timestampsWindow[:-1]
                nbMsgDropped = np.count_nonzero(timeDiffs > np.mean(timeDiffs)*(1.0 + dropThreshold))
                totalDropOverTime[i] += nbMsgDropped

    duration =  lastMessageStamp - firstMessageStamp
    return totalDropOverTime, firstMessageStamp, duration

def findBestDataWindow(drops, startTime, duration, T=600):

    if T > duration:
        raise Exception("Crop window size (%f sec) too small for data (duration %f sec" % (T, duration))

    subWindowWidth = duration / len(drops)
    if T >= subWindowWidth:
        W = np.ones(int(np.ceil(T/subWindowWidth)))
        # conv will be len(graph) - len(W) long. The valid argument
        #specifies only entirely overlapping results to be kept
        conv = np.convolve(drops, W, mode='valid')
        pos = np.argmin(conv)
        cenPos = (len(W)/2 + pos) * subWindowWidth
        cenPosAbs = cenPos + startTime 

    return cenPos, cenPosAbs
