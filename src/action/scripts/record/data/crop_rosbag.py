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

import os
import logging
import numpy as np
import rosbag

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from optparse import OptionParser
from bagutils import getAllTopicsMetadata, getDropsTimeDistribution, findBestDataWindow, plotDropsTimeDistribution

logger = logging.getLogger(__name__)

def getCropTimes(topicTimestamps, dropThreshold=1.0, windowSize=600, ignoreBorder=15, windowConv=10):
    drops, startRefTime, recordDuration = getDropsTimeDistribution(topicTimestamps, dropThreshold, windowWidth=windowConv, ignoreBorder=ignoreBorder)
    centerPos, centerPosAbs = findBestDataWindow(drops, startRefTime, recordDuration, T=windowSize)
    
    startTime = centerPosAbs - windowSize/2
    endTime = centerPosAbs + windowSize/2
    return startTime, endTime, drops, centerPos

def cropRosbag(inbagFile, outbagFile, startTime, endTime, useRosbagTime=False):
    with rosbag.Bag(outbagFile, 'w') as outbag:
        for topic, msg, timestamp in rosbag.Bag(inbagFile).read_messages():
            if not useRosbagTime:
                # Get timestamp from header
                timestamp = msg.header.stamp
            
            t = float(timestamp.to_sec())
            if t >= startTime and t <= endTime:
                outbag.write(topic, msg, timestamp)

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=False,
                      help="Location of Rosbag to analyse")
    parser.add_option("-o", "--output",  dest="output", default="",
                      help="Output rosbag file")
    parser.add_option("-r", "--ignore-topics", dest="topicRemove", default="",
                      help="Comma separated list of topics to ignore")
    parser.add_option("-c", "--use-rosbag-time",
                      action="store_true", dest="useRosbagTime", default=False,
                      help="Use rosbag time instead of capture time")
    parser.add_option("-m", "--simulate",
                      action="store_true", dest="simulate", default=False,
                      help="Simulate the cropping, without actually writing the output rosbag")
    parser.add_option("-t", "--drop-threshold", dest="dropThreshold", type='float', default=1.0,
                      help='Specify the threshold to use for detecting dropped messages')
    parser.add_option("-e", "--crop-window", dest="cropWindow", type='int', default=600, help="Specify in seconds the windows of best quality data to extract")

    parser.add_option("-p", "--ignore-border", dest="ignoreBorder", type='int', default=15, help="Specify in seconds the time to ignore at the start and end of rosbag")

    parser.add_option("-w", "--window-size-conv", dest="windowSizeConv", type='int', default=10, help="Specify in seconds the window to use for convolution")

    parser.add_option("-s", "--save-drop-distribution", dest="saveDropDistribution", default="", help="Specify to save the drop time distribution figure to file")
    (options, args) = parser.parse_args()

    if not options.input:
        parser.error("Please specify input rosbag file with -i ")
    inputRosbagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (inputRosbagPath))

    if not options.output:
        parser.error("Please specify output rosbag file with -o ")
    outputRosbagFilePath = os.path.abspath(options.output)
    logger.info('Using output rosbag file: %s' % (outputRosbagFilePath))

    ignoreList = []
    if options.topicRemove:
        ignoreList = options.topicRemove.split(',')
        logger.info('Ignored topics: %s' % (options.topicRemove))

    # Get timestamps and sequence ids from the rosbag
    topicTimestamps, topicSequenceIds = getAllTopicsMetadata(inputRosbagPath, ignoreList, options.useRosbagTime)

    if options.cropWindow <= 0:
        raise Exception('Specified crop window size must be greater than zero')

    startTime, endTime, drops, centerPos = getCropTimes(topicTimestamps,
                                                        options.dropThreshold, options.cropWindow, 
                                                        options.ignoreBorder, options.windowSizeConv)
    logger.info('Using calculated crop times: start = %f, end = %f' % (startTime, endTime))
    
    if options.saveDropDistribution:
        outputFigureFilePath = os.path.abspath(options.saveDropDistribution)
        
        fig = plotDropsTimeDistribution(drops,
                                        centerPos=centerPos,
                                        cropWindow=options.cropWindow,
                                        windowSizeConv=options.windowSizeConv)
        
        logger.info('Saving drop time distribution figure to file: %s' % (outputFigureFilePath))
        plt.savefig(outputFigureFilePath, dpi=100)
        plt.close(fig)

    if not options.simulate:
        cropRosbag(inputRosbagPath, outputRosbagFilePath, startTime, endTime)
    else:
        logger.info('Skipping cropping because simulation mode is activated')

    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
