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

from stats_rosbag import getAllTopicsMetadata, getdroprateGraphOverTime, findBestDataWindow


from optparse import OptionParser

logger = logging.getLogger(__name__)


def getExtractionTimes(topicTimestamps, dropThreshold=1.0, windowSize=600):

    droppedMsgsOT, startTime, recordDuration = getdroprateGraphOverTime(topicTimestamps, dropThreshold)

    centerPos, centerPosAbs = findBestDataWindow(droppedMsgsOT, startTime, recordDuration, T=windowSize)

    start = centerPosAbs - windowSize/2
    end = centerPosAbs + windowSize/2
    return start , end

def extractRosbag(inbagFile, outbagFile, start, end):
    with rosbag.Bag(outbagFile, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(inbagFile).read_messages():
            # This also replaces tf timestamps under the assumption
            # that all transforms in the message share the same timestamp
            t_sec = t.to_sec()
            if t_sec >= start and t_sec <= end:
                outbag.write(topic, msg, t)

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
    parser.add_option("-t", "--drop-threshold", dest="dropThreshold", type='float', default=1.0,
                      help='Specify the threshold to use for detecting dropped messages')
    parser.add_option("-e", "--extract", dest="extractDataDuration", type='int', default=-1, help="Specify in seconds the windows of best quality data to extract")
    (options, args) = parser.parse_args()

    if not options.input:
        parser.error("Please specify input rosbag file with -i ")

    inputRosbagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (inputRosbagPath))

    if options.output:
        outputRosbagFilePath = os.path.abspath(options.output)
        logger.info('Using output rosbag file: %s' % (outputRosbagFilePath))

    ignoreList = []
    if options.topicRemove:
        ignoreList = options.topicRemove.split(',')
        logger.info('Ignored topics: %s' % (options.topicRemove))

    # Get timestamps and sequence ids from the rosbag
    topicTimestamps, topicSequenceIds = getAllTopicsMetadata(inputRosbagPath, ignoreList, options.useRosbagTime)


    if options.extractDataDuration >= 0 and options.output:
        start, end = getExtractionTimes(topicTimestamps, options.dropThreshold, options.extractDataDuration)

        extractRosbag(inputRosbagPath, outputRosbagFilePath, start, end)




    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
