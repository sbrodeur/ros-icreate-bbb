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

from tabulate import tabulate
from optparse import OptionParser

logger = logging.getLogger(__name__)

def getStatisticsFromTimestamps(timestamps, dropThreshold=1.0):
    
    # Calculate relative times between messages
    timeDiffs = timestamps[1:] - timestamps[:-1]
    mean = np.mean(timeDiffs)
    variance = np.var(timeDiffs)
    averageRate = 1.0/mean
    nbMsgs = len(timestamps)
    nbMsgDropped = np.count_nonzero(timeDiffs > mean*(1.0 + dropThreshold))
    
    return mean, variance, averageRate, nbMsgs, nbMsgDropped

def getTabulatedStatistics(topicTimestamps, dropThreshold):

    headers = ["Topic name", "Average Rate [Hz]", "Average Period [ms]",
               "Std Deviation Period [ms]", "Nb of Messages", "Drop ratio [%]"]
    
    # Loop for each topic
    data = []
    for topic, timestamps in topicTimestamps.iteritems():
        
        # Calculate statistics
        mean, variance, averageRate, nbMsgs, nbMsgDropped = getStatisticsFromTimestamps(timestamps, dropThreshold)
        
        # Add to table
        data.append([topic, averageRate, mean*1000.0, np.sqrt(variance)*1000.0, nbMsgs, nbMsgDropped/float(nbMsgs) * 100.0])

    # Sort table by topic name
    data.sort(key=lambda x: x[0])
    
    return tabulate(data, headers, tablefmt="grid", numalign="right", stralign="left", floatfmt=".2f")
  
def getAllTopicTimestamps(filename, ignoredTopics, useRosbagTime=False):
    
    topicTimestamps = dict()
    with rosbag.Bag(filename) as bag:
        # Grab list of topics
        topics = bag.get_type_and_topic_info()[1].keys()
        
        # Generate Topic statisitics objects
        for topic in topics:
            if not topic in ignoredTopics:
                topicTimestamps[topic] = []

        # Process all messages
        for topic, msg, t in bag.read_messages():
            if topic in topicTimestamps:
                if useRosbagTime:
                    t = t.to_sec()
                else:
                    # Get timestamp from header
                    t = float(msg.header.stamp.to_sec())
                topicTimestamps[topic].append(t)
                
    # Convert to sorted numpy arrays
    # NOTE: sorting is important when using capture time, since they are not guaranteed
    #       to be ordered as in the rosbag.
    for topic, timestamps in topicTimestamps.iteritems():
        topicTimestamps[topic] = np.sort(np.array(timestamps))
    return topicTimestamps

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=False,
                      help="Location of Rosbag to analyse")
    parser.add_option("-o", "--output",  dest="output", default="",
                      help="Output stats file")
    parser.add_option("-r", "--ignore-topics", dest="topicRemove", default="",
                      help="Comma separated list of topics to ignore")
    parser.add_option("-c", "--use-rosbag-time",
                      action="store_true", dest="useRosbagTime", default=False,
                      help="Use rosbag time instead of capture time")
    parser.add_option("-t", "--drop-threshold", dest="dropThreshold", type='float', default=1.0,
                      help='Specify the threshold to use for detecting dropped messages')
    (options, args) = parser.parse_args()

    if not options.input:
        parser.error("Please specify input rosbag file with -i ")
    
    inputRosbagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (inputRosbagPath))

    if options.output:
        outputStatsFilePath = os.path.abspath(options.output)
        logger.info('Using output statistics file: %s' % (outputStatsFilePath))
    
    ignoreList = []
    if options.topicRemove:
        ignoreList = options.topicRemove.split(',')
        logger.info('Ignored topics: %s' % (options.topicRemove))
    
    # Get timestamps from the rosbag
    topicTimestamps = getAllTopicTimestamps(inputRosbagPath, ignoreList, options.useRosbagTime)
    
    # Print statistics to string
    str = getTabulatedStatistics(topicTimestamps, options.dropThreshold)
    
    if not options.output:
        # Just print to stdout
        print str
    else:
        # Print to file
        with open(outputStatsFilePath, 'w') as f:
            f.write(str + '\n')
    
    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
