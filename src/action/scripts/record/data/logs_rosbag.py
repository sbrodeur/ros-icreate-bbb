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
import rosbag

from cStringIO import StringIO
from optparse import OptionParser

logger = logging.getLogger(__name__)

def convertMsgToCSV(msg, separator=';'):
    # header is "message;severity;node;stamp;topics;location"
    
    # message, severity, node
    items = ['"' + str(msg.msg) + '"', '"' + str(msg.level) + '"', '"' + str(msg.name) + '"']
    
    # stamp
    items.append('"' + str(msg.header.stamp.secs) + '.' + str(msg.header.stamp.nsecs) + '"')
    
    # topics
    items.append('"' + ",".join(msg.topics) + '"')
    
    # location
    items.append('"' + str(msg.file) + ':' + str(msg.function) + ':' + str(msg.line) + '"')
    
    return separator.join(items)

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=False,
                      help="Location of Rosbag to analyse")
    parser.add_option("-o", "--output",  dest="output", default="",
                      help="Output log file")
    (options, args) = parser.parse_args()

    if not options.input:
        parser.error("Please specify input rosbag file with -i ")
    
    inputRosbagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (inputRosbagPath))

    if options.output:
        outputLogFilePath = os.path.abspath(options.output)
        logger.info('Using output log file: %s' % (outputLogFilePath))

    # Use to build string efficiently
    s = StringIO()
    
    # Write header
    s.write('message;severity;node;stamp;topics;location' + '\n')

    with rosbag.Bag(inputRosbagPath) as bag:

        # Write all messages from the rosout topic
        nbMsgFound = 0
        for topic, msg, t in bag.read_messages(topics=['/rosout']):
            msgstr = convertMsgToCSV(msg)
            s.write(msgstr + '\n')
            nbMsgFound += 1
    
    logger.info('Found %d log messages to convert' % (nbMsgFound))
    
    str = s.getvalue()
    
    if not options.output:
        # Just print to stdout
        print str
    else:
        # Print to file
        with open(outputLogFilePath, 'w') as f:
            f.write(str)
    
    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
