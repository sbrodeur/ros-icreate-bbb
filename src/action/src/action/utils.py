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
import time
import logging
import numpy as np
import rospy
import subprocess
import atexit

logger = logging.getLogger(__name__)

class RosManager:
    
    ROS_HOSTNAME='localhost'
    ROS_MASTER_URI='http://localhost:1234/'
    ROS_PORT = 1234
    
    def __init__(self, rosbagPath=None, rate=1):
        self.rate = rate
        self._terminate()
        
        # Start new roscore instance
        logger.info('Starting roscore subprocess...')
        env = os.environ
        env['ROS_HOSTNAME'] = RosManager.ROS_HOSTNAME
        env['ROS_MASTER_URI'] = self.ROS_MASTER_URI
        env['CMAKE_PREFIX_PATH'] = '/opt/ros/indigo'
        proc_roscore = subprocess.Popen(['sh /opt/ros/indigo/setup.sh & roscore -p %d' % (self.ROS_PORT)],
                                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                                             shell=True, env=env)
        
        time.sleep(1.0)
        if proc_roscore.poll() is not None:
            lines = proc_roscore.communicate()[1]
            raise Exception('Failed to start the roscore subprocess: \n%s' % (lines))
        logger.info('roscore successfully launched!')
        atexit.register(self.close)
        
        # Start new rosbag instance, if a file was provided
        self.rosbagPath = None
        if rosbagPath is not None:
            rosbagPath = os.path.abspath(rosbagPath)
            self.rosbagPath = rosbagPath
            logger.info('Starting rosbag subprocess using file %s ...' % (rosbagPath))
            proc_rosbag = subprocess.Popen(['rosbag play -q -r %d %s' % (int(rate), rosbagPath)], 
                                                stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                                                shell=True, env=env)
            
            time.sleep(1.0)
            if proc_rosbag.poll() is not None:
                lines = proc_rosbag.communicate()[1]
                raise Exception('Failed to start the rosbag subprocess: \n%s' % (lines))
            logger.info('rosbag successfully launched!')
        
        rospy.init_node('test', anonymous=True, log_level=rospy.DEBUG)
        logger.info('rospy initialization done!')
    
    def launch(self, filepath):
        
        env = os.environ
        env['ROS_HOSTNAME'] = RosManager.ROS_HOSTNAME
        env['ROS_MASTER_URI'] = self.ROS_MASTER_URI
        env['CMAKE_PREFIX_PATH'] = '/opt/ros/indigo'
        
        filepath = os.path.abspath(filepath)
        logger.info('Starting roslauch subprocess for file %s ...' % (filepath))
        proc_roslaunch = subprocess.Popen(['sh ..//setup.sh & roslaunch %s' % (filepath)], 
                                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, 
                                            shell=True, env=env)
        
        time.sleep(1.0)
        if proc_roslaunch.poll() is not None:
            lines = proc_roslaunch.communicate()[1]
            raise Exception('Failed to start the roslaunch subprocess: \n%s' % (lines))
        logger.info('roslaunch successfully launched!')
        
    def _terminate(self):
        
        # Kill any existing roscore process
        # Source: http://carpenoctem.das-lab.net/~cn/snapshotRepository130910/impera/TestPackages/TestSolver/src/RestartRoscore.py
        proc = subprocess.Popen(['ps', 'x'], shell=False, stdout=subprocess.PIPE)
        lines = proc.communicate()[0].split('\n')
        proc.wait()
        for line in lines:
            if (line.find("roscore") > -1 or line.find("rosmaster") >-1 or 
                line.find("rosout") > -1 or line.find("rosbag") > -1):
                pid = line.lstrip(" \t\n").split(' ')[0]
                logger.info('Found a roscore/rosmaster/rosout/rosbag instance: killing pid %s' % (pid))
                p = subprocess.Popen(['kill','-9', pid]);
                p.wait()
        
    def close(self):
        logger.info('Terminating roscore instance.')
        self._terminate()


def configureRootLogger(level=logging.DEBUG, logFile=None, overwriteLogFile=True):
    """
    Configure the root logger that will output the logs to console (STDOUT) and optionally to file.
    
    :param level: the minimum log level to display. Default is DEBUG.
    :param logFile: the filename of log file to write. The content will be a copy of what is shown in the console.
    :param overwriteLogFile: force to overwrite any existing log file. Default is to True.
    """
    logger = logging.getLogger()
    logger.setLevel(level)
    
    # Remove existing handlers
    if logging.root:
        del logging.root.handlers[:]

    # Custom formatter
    formatter = logging.Formatter('[%(levelname)s] [%(asctime)s] - %(message)s')
    
    if logFile is not None:
        # File logger
        if os.path.exists(logFile) and overwriteLogFile:
            os.remove(logFile)
        fh = logging.FileHandler(os.path.abspath(logFile))
        fh.setLevel(level)
        fh.setFormatter(formatter)
        logger.addHandler(fh)

    # Console logger
    ch = logging.StreamHandler()
    ch.setLevel(level)
    ch.setFormatter(formatter)
    logger.addHandler(ch)


def executeSystemCommand(command, name=None, realtimeLog=False, level=logging.DEBUG):
    """
    Execute a system command on the shell.
    
    :param command: the command to execute.
    :param name: the short name of the command that can be shown in the log output.
    :param realtimeLog: set whether or not the log should be accumulated (False) or output in real-time (True). Default is to False.
    :param level: the log level for the standard output of the command. Default is DEBUG.
    """
    logging.debug('Executing system command: %s' % (command))
    
    # Launch command in a subprocess
    p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if name is not None:
        logging.log(level,"################# %s #################" % (name))
        
    lines = []
    for line in iter(p.stdout.readline, ''):
        line = line.replace('\r', '').replace('\n', '')
        if realtimeLog:
            logging.log(level, line)
        else:
            lines.append(line)
        
    if not realtimeLog:
        logging.log(level, '\n'.join(lines))
        
    if name is not None:
        logging.log(level, "############################################################")
    retval = p.wait()
    assert(retval == 0)

