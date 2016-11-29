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

from tabulate import tabulate
from optparse import OptionParser
from bagutils import getAllTopicsMetadata, findBestDataWindow, getDropsTimeDistribution, plotDropsTimeDistribution

logger = logging.getLogger(__name__)

def getStatisticsFromTimestamps(timestamps, dropThreshold=1.0):

    # Calculate relative times between messages
    timeDiffs = timestamps[1:] - timestamps[:-1]
    mean = np.mean(timeDiffs)
    variance = np.var(timeDiffs)
    if mean > 0.0:
        averageRate = 1.0/mean
    else:
        averageRate = 0.0
    nbMsgs = len(timestamps)
    nbMsgDropped = np.count_nonzero(timeDiffs > mean*(1.0 + dropThreshold))

    return mean, variance, averageRate, nbMsgs, nbMsgDropped

def saveHistogramsToFiles(topicTimestamps, dropThreshold, outputPath):

    outputPath = os.path.abspath(outputPath)
    if not os.path.exists(outputPath):
        logger.info('Creating output directory for histograms: %s' % (outputPath))
        os.makedirs(outputPath)

    for topic, timestamps in topicTimestamps.iteritems():
        name = topic[1:].replace("/", "-")

        # Compute time delays (in msec)
        timeDiffs = (timestamps[1:] - timestamps[:-1]) * 1000.0
        mean = np.mean(timeDiffs)

        fig = plt.figure(figsize=(8,6), facecolor='white')

        plt.title("Histogram for " + name)
        plt.xlabel('Time delay between messages [ms]')
        plt.ylabel('Message count')
        plt.hist(timeDiffs, bins=500, color='k')
        plt.axvline(mean, color='b')
        plt.axvline(mean + mean*dropThreshold, color='r')
        plt.axvline(mean - mean*dropThreshold, color='r')

        filename = os.path.join(outputPath, name + '.png')
        logger.info('Saving histogram figure to file: %s' % (filename))
        plt.savefig(filename, dpi=100)
        plt.close(fig)

def saveDropMsgsOverTime(topicTimestamps, dropThreshold, outputPath, windowSize=600):
    outputPath = os.path.abspath(outputPath)
    if not os.path.exists(outputPath):
        logger.info('Creating output directory for histograms: %s' % (outputPath))
        os.makedirs(outputPath)

    droppedMsgsOT, startTime, recordDuration = getDropRatesOverTime(topicTimestamps, dropThreshold)

    centerPos, centerPosAbs = findBestDataWindow(droppedMsgsOT, startTime, recordDuration, T=windowSize)

    fig = plotDropsTimeDistribution(droppedMsgsOT, centerPos, windowSize)

    filename = os.path.join(outputPath, 'msgs_dropped.png')
    logger.info('Saving histogram figure to file: %s' % (filename))
    plt.savefig(filename, dpi=100)
    plt.close(fig)

def getTabulatedStatistics(topicTimestamps, topicSequenceIds, dropThreshold):

    headers = ["Topic name", "Average Rate [Hz]", "Average Period [ms]",
               "Std Deviation Period [ms]", "Nb of Messages", "Drop ratio [%] (timestamp-based)", "Drop ratio [%] (sequence-based)"]

    # Loop for each topic
    data = []
    for topic, timestamps in topicTimestamps.iteritems():

        # Calculate statistics from timestamps
        mean, variance, averageRate, nbMsgs, nbMsgDropped = getStatisticsFromTimestamps(timestamps, dropThreshold)

        # Calculate statistics from sequence ids
        ids = topicSequenceIds[topic]
        minId, maxId = np.min(ids), np.max(ids)
        nbMsgsSeqId = (maxId + 1) - minId
        diff = np.setdiff1d(ids, np.arange(minId, maxId+1, dtype=np.int))
        nbMsgDroppedSeqId = len(diff)

        # Add to table
        data.append([topic, averageRate, mean*1000.0, np.sqrt(variance)*1000.0, nbMsgs, nbMsgDropped/float(nbMsgs) * 100.0, nbMsgDroppedSeqId/float(nbMsgsSeqId) * 100.0])

    # Sort table by topic name
    data.sort(key=lambda x: x[0])

    return tabulate(data, headers, tablefmt="grid", numalign="right", stralign="left", floatfmt=".2f")

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
    parser.add_option("-g", "--save-histograms",
                      action="store_true", dest="saveHistograms", default=False,
                      help="Specify to save the histograms as images")
    parser.add_option("-d", "--save-dropped",
                      action="store_true", dest="saveDropped", default=False,
                      help="Specify to save the dropped messages graph as images")
    parser.add_option("-t", "--drop-threshold", dest="dropThreshold", type='float', default=1.0,
                      help='Specify the threshold to use for detecting dropped messages')
    (options, args) = parser.parse_args()

    if not options.input:
        parser.error("Please specify input rosbag file with -i ")

    if (options.saveHistograms or options.saveDropped)and not options.output:
        parser.error("Please specify an output file with -o when exporting histograms ")

    inputRosbagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (inputRosbagPath))

    if options.output:
        outputStatsFilePath = os.path.abspath(options.output)
        logger.info('Using output statistics file: %s' % (outputStatsFilePath))

    ignoreList = []
    if options.topicRemove:
        ignoreList = options.topicRemove.split(',')
        logger.info('Ignored topics: %s' % (options.topicRemove))

    # Get timestamps and sequence ids from the rosbag
    topicTimestamps, topicSequenceIds = getAllTopicsMetadata(inputRosbagPath, ignoreList, options.useRosbagTime)

    if options.saveHistograms and options.output:
        outputPath = outputStatsFilePath + '.histograms'
        saveHistogramsToFiles(topicTimestamps, options.dropThreshold, outputPath)

    if options.saveDropped and options.output:
        outputPath = outputStatsFilePath + '.drop'
        saveDropMsgsOverTime(topicTimestamps, options.dropThreshold, outputPath, windowSize=600)

    # Print statistics to string
    str = getTabulatedStatistics(topicTimestamps, topicSequenceIds, options.dropThreshold)

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
