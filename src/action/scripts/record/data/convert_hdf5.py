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

import sys
import os
import logging
import itertools
import h5py
import numpy as np
from optparse import OptionParser

import rospy
import rosbag

logger = logging.getLogger(__name__)

class StateSaver(object):
    
    def __init__(self, outHdf5):
        self.outHdf5 = outHdf5
        
        self.ignoredTopics = []
        self.countTotal = 0
        self.countProcessed = 0
        
    def add(self, topic, msg, t):
        self._process(topic, msg, t)
        self.countTotal += 1
    
    def _process(self, topic, msg, t):
        
        lastProcessed = self.countProcessed
        
        if topic == '/imu/data':
            
            # Orientation
            state = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=np.float64)
            self.outHdf5.addState('orientation', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Angular velocity
            state = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64)
            self.outHdf5.addState('angular_velocity', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Linear acceleration
            state = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=np.float64)
            self.outHdf5.addState('linear_acceleration', state, t, group='imu', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/imu/data_raw':
            
            # Orientation
            state = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w], dtype=np.float64)
            self.outHdf5.addState('orientation_raw', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Angular velocity
            state = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z], dtype=np.float64)
            self.outHdf5.addState('angular_velocity_raw', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Linear acceleration
            state = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z], dtype=np.float64)
            self.outHdf5.addState('linear_acceleration_raw', state, t, group='imu', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/imu/mag':
            
            # Magnetic field
            state = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z], dtype=np.float64)
            self.outHdf5.addState('magnetic_field', state, t, group='imu', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/imu/temp':

            # Temperature
            state = np.array([msg.temperature], dtype=np.float64)
            self.outHdf5.addState('temperature', state, t, group='imu', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/irobot_create/battery':
            
            # State
            state = np.array([msg.voltage, msg.current, msg.charge, msg.capacity, msg.design_capacity, msg.percentage], dtype=np.float32)
            self.outHdf5.addState('charge', state, t, group='battery', variableShape=False, maxShape=None)
            
            # Status
            state = np.array([msg.power_supply_status, msg.power_supply_health, msg.power_supply_technology], dtype=np.uint8)
            self.outHdf5.addState('status', state, t, group='battery', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
        
        if topic == '/video/left/compressed':
            state = np.fromstring(msg.data, dtype=np.uint8)
            self.outHdf5.addState('left', state, t, group='video', variableShape=True, maxShape=(32768,))
            self.countProcessed += 1
        
        if topic == '/video/right/compressed':
            state = np.fromstring(msg.data, dtype=np.uint8)
            self.outHdf5.addState('right', state, t, group='video', variableShape=True, maxShape=(32768,))
            self.countProcessed += 1
            
        if topic == '/audio/left/raw':
            state = np.array(msg.data, dtype=np.int16)
            self.outHdf5.addState('left', state, t, group='audio', variableShape=False, maxShape=None)
            self.countProcessed += 1
        
        if topic == '/audio/right/raw':
            state = np.array(msg.data, dtype=np.int16)
            self.outHdf5.addState('right', state, t, group='audio', variableShape=False, maxShape=None)
            self.countProcessed += 1
        
        if topic == '/irobot_create/contact':
            
            # Collision and cliff detection (binary)
            state = np.array([msg.bumpLeft, msg.bumpRight,
                              msg.wheeldropCaster, msg.wheeldropLeft, msg.wheeldropRight,
                              msg.cliffLeft, msg.cliffFrontLeft, msg.cliffFrontRight, msg.cliffRight,
                              msg.wall, msg.virtualWall], dtype=np.uint8)
            self.outHdf5.addState('switch', state, t, group='collision', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/irobot_create/irRange':

            # Collision and cliff detection (range)
            state = np.array([msg.wallSignal, msg.cliffLeftSignal, msg.cliffFrontLeftSignal,
                              msg.cliffFrontRightSignal, msg.cliffRightSignal], dtype=np.uint16)
            self.outHdf5.addState('range', state, t, group='collision', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
        
        if topic == '/irobot_create/motorRequests':
            # WARNING: deprecated message
            
            # Motor velocity
            state = np.array([msg.requestedLeftVelocity, msg.requestedRightVelocity], dtype=np.int16)
            self.outHdf5.addState('speed', state, t, group='motors', variableShape=False, maxShape=None)
            
            self.countProcessed += 1

        if topic == '/irobot_create/motors':
            
            # Motor velocity
            state = np.array([msg.left, msg.right], dtype=np.int16)
            self.outHdf5.addState('speed', state, t, group='motors', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
        
        if topic == '/irobot_create/odom':

            # Odometry position
            position = msg.pose.pose.position
            state = np.array([position.x, position.y, position.z], dtype=np.float64)
            self.outHdf5.addState('position', state, t, group='odometry', variableShape=False, maxShape=None)
            
            # Odometry orientation
            orientation = msg.pose.pose.orientation
            state = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=np.float64)
            self.outHdf5.addState('orientation', state, t, group='odometry', variableShape=False, maxShape=None)
            
            # Odometry twist (linear)
            twist_linear = msg.twist.twist.linear
            state = np.array([twist_linear.x, twist_linear.y, twist_linear.z], dtype=np.float64)
            self.outHdf5.addState('twist_linear', state, t, group='odometry', variableShape=False, maxShape=None)
            
            # Odometry twist (angular)
            twist_angular = msg.twist.twist.angular
            state = np.array([twist_angular.x, twist_angular.y, twist_angular.z], dtype=np.float64)
            self.outHdf5.addState('twist_angular', state, t, group='odometry', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
        
        # Check for ignored message topics
        if lastProcessed == self.countProcessed and topic not in self.ignoredTopics:
            logger.info('Ignoring messages from topic: %s' % (topic))
            self.ignoredTopics.append(topic)
            
        if self.countProcessed > 0 and self.countProcessed % 100 == 0:
            logger.info('Processed %d messages out of %d total messages' % (self.countProcessed, self.countTotal))
        
class Hdf5Bag:

    def __init__(self, filePath, chunkSize=32, overwrite=False):
        self.chunkSize = chunkSize
        
        # Possible optimization: libver='latest'
        if overwrite:
            # Create file, truncate if exists
            self.h = h5py.File(filePath, mode='w')
        else:
            # Read/write if exists, create otherwise (default)
            self.h = h5py.File(filePath, mode='a')
        
        self.datasets = {}
        self.nbStateSamples = {}
    
    def addState(self, name, state, t, group=None, variableShape=False, maxShape=None):
        
        if group is not None:
            baseName = '/' + group + '/' + name
        else:
            baseName = '/' + name
        
        datasetName = baseName + '/raw'
        clockName = baseName + '/clock'
        shapeName = baseName + '/shape'
        
        if datasetName not in self.datasets:
        
            if group is not None:
                try:
                    file = self.h.create_group(group)
                except ValueError:
                    file = self.h[group]
            else:
                file = self.h
        
            group = file.create_group(name)
        
            # NOTE: we need to create a dataset for each state once knowing the shape of the state,
            #       so the compression can use adequate chunk size
            if variableShape:
                dataShape = maxShape
            else:
                dataShape = state.shape

            dataset = group.create_dataset('raw',
                                            shape=(self.chunkSize,) + dataShape, maxshape=(None,) + dataShape, 
                                            dtype=state.dtype,
                                            chunks=True,
                                            compression='gzip', compression_opts=6, shuffle=True)
            self.datasets[dataset.name] = dataset
            self.nbStateSamples[dataset.name] = 0
            
            clockDataset = group.create_dataset('clock',
                                                 shape=(self.chunkSize,), maxshape=(None,), 
                                                 dtype=np.float64)
            self.datasets[clockDataset.name] = clockDataset
            self.nbStateSamples[clockDataset.name] = 0
            
            if variableShape:
                shapeDataset = group.create_dataset('shape',
                                                    shape=(self.chunkSize, state.ndim), maxshape=(None, state.ndim), 
                                                    dtype=np.int64)
                
                self.datasets[shapeDataset.name] = shapeDataset
                self.nbStateSamples[shapeDataset.name] = 0
        else:
            dataset = self.datasets[datasetName]
            clockDataset = self.datasets[clockName]
            if variableShape:
                shapeDataset = self.datasets[shapeName]
            
        nbStateSamples = self.nbStateSamples[dataset.name]
        
        # Resize dataset if necessary
        if nbStateSamples >= dataset.shape[0]:
            newShape = np.ceil(float(nbStateSamples + 1) / self.chunkSize) * self.chunkSize
            dataset.resize(newShape, axis=0)
            clockDataset.resize(newShape, axis=0)
            if variableShape:
                shapeDataset.resize(newShape, axis=0)
            self.h.flush()
            
        # Append state data to dataset
        if variableShape:
            slices = (nbStateSamples,)
            for i in state.shape:
                slices += (slice(0, i),)
            dataset[slices] = state
            shapeDataset[nbStateSamples] = state.shape
        else:
            dataset[nbStateSamples] = state
        clockDataset[nbStateSamples] = t
        
        self.nbStateSamples[dataset.name] += 1
        self.nbStateSamples[clockDataset.name] += 1
        if variableShape:
            self.nbStateSamples[shapeDataset.name] += 1
            
    def close(self):
        for name, dataset in self.datasets.iteritems():
            # Truncate dataset to the actual number of state samples
            dataset.resize(self.nbStateSamples[name], axis=0)
        self.h.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input bag file')
    parser.add_option("-o", "--output", dest="output", default=None,
                      help='specify the path of the output HDF5 file')
    parser.add_option("-b", "--start-time", dest="startTime", default=0.0,
                      help='specify the minimum time')
    parser.add_option("-e", "--stop-time", dest="stopTime", default=-1.0,
                      help='specify the maximum time')
    parser.add_option("-k", "--chunk-size", dest="chunkSize", default=128,
                      help='specify the chunk size')
    parser.add_option("-c", "--use-capture-time",
                      action="store_true", dest="useCaptureTime", default=False,
                      help="use capture time rather than recorded time")
    parser.add_option("-r", "--use-relative-time",
                      action="store_true", dest="useRelativeTime", default=False,
                      help="use time relative to the first message")
    (options,args) = parser.parse_args(args=args)

    rosBagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (rosBagPath))

    datasetPath = os.path.abspath(options.output)    
    logger.info('Using output hdf5 file: %s' % (datasetPath))
    
    startTime = float(options.startTime)
    stopTime = float(options.stopTime)
    with Hdf5Bag(datasetPath, chunkSize=int(options.chunkSize), overwrite=True) as outHdf5:
        
        referenceTime = None
        if options.useRelativeTime:
            with rosbag.Bag(rosBagPath) as inbag:
                if options.useCaptureTime:
                    # Use the smallest timestamp amonsgt the first 100 messages of each topic
                    MAX_COUNT = 100
                    topics = sorted(set([c.topic for c in inbag._get_connections()]))
                    timestamps = []
                    for topic in topics:
                        minTimestamp = None
                        count = 0
                        for _, msg, _ in inbag.read_messages(topics=[topic]):
                            if hasattr(msg, 'header'):
                                t = float(msg.header.stamp.to_sec())
                                if minTimestamp is None or t < minTimestamp:
                                    minTimestamp = t
                                count += 1
                                if count >= MAX_COUNT:
                                    break
                            break
                        if minTimestamp is not None:
                            timestamps.append(minTimestamp)
                    referenceTime = np.min(timestamps)
                    print referenceTime, timestamps
                else:
                    # Use the timestamp of the first recorded message
                    for _, _, timestamp in inbag.read_messages():
                        referenceTime = float(timestamp.to_sec())
                        break
            logger.info('Using reference time: %f' % (referenceTime))
        
        with rosbag.Bag(rosBagPath) as inbag:
            
            saver = StateSaver(outHdf5)
            ignoredTopics = ['/rosout', '/rosout_agg', '/tf',
                             '/video/left/camera_info', '/video/right/camera_info',
                             '/madgwick/parameter_descriptions',
                             '/madgwick/parameter_updates']
            
            for topic, msg, timestamp in inbag.read_messages():
                
                # Skip ignored topics
                if topic in ignoredTopics:
                    continue
                
                if options.useCaptureTime:
                    if hasattr(msg, 'header'):
                        # Use header timestamp
                        t = float(msg.header.stamp.to_sec())
                    else:
                        raise Exception('Topic %s has no header: timestamps will be estimated from rosbag' % (topic))
                else:
                    # Use rosbag recorded timestamp
                    t = float(timestamp.to_sec())
                
                if options.useRelativeTime:
                    # Make clock time relative to the first timestamp
                    t = t - referenceTime
                    assert t >= 0.0
                
                # Validate if within timerange
                if t < startTime:
                    continue

                print t                
                saver.add(topic, msg, t)
                
                if stopTime >= 0.0 and t > stopTime:
                    logger.info('Stop time (%f sec) reached' % (stopTime))
                    break
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    