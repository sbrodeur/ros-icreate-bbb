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
        
        if topic == '/imu/raw':
            # Accelerometer
            state = np.array([msg.accelX, msg.accelY, msg.accelZ], dtype=np.float32)
            self.outHdf5.addState('accel', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Magnetometer
            state = np.array([msg.magX, msg.magY, msg.magZ], dtype=np.float32)
            self.outHdf5.addState('mag', state, t, group='imu', variableShape=False, maxShape=None)
            
            # Gyroscope
            state = np.array([msg.gyroX, msg.gyroY, msg.gyroZ], dtype=np.float32)
            self.outHdf5.addState('gyro', state, t, group='imu', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        if topic == '/irobot_create/sensorPacket':
            # Rescale velocities according to max absolute value (500 mm/sec)
            leftVel = float(msg.requestedLeftVelocity) / 500.0
            rightVel = float(msg.requestedRightVelocity) / 500.0
            state = np.array([leftVel, rightVel], dtype=np.float32)
            self.outHdf5.addState('motors', state, t, group=None, variableShape=False, maxShape=None)
            self.countProcessed += 1
        
        if topic == '/video/left/raw/compressed':
            state = np.fromstring(msg.data, dtype=np.uint8)
            self.outHdf5.addState('left', state, t, group='video', variableShape=True, maxShape=(32768,))
            self.countProcessed += 1
        
        if topic == '/video/right/raw/compressed':
            state = np.fromstring(msg.data, dtype=np.uint8)
            self.outHdf5.addState('right', state, t, group='video', variableShape=True, maxShape=(32768,))
            self.countProcessed += 1
            
        if topic == '/audio/left/raw':
            state = np.array(msg.data, dtype=np.int16).astype(np.float32) / np.iinfo(np.int16).max
            self.outHdf5.addState('left', state, t, group='audio', variableShape=False, maxShape=None)
            self.countProcessed += 1
        
        if topic == '/audio/right/raw':
            state = np.array(msg.data, dtype=np.int16).astype(np.float32) / np.iinfo(np.int16).max
            self.outHdf5.addState('right', state, t, group='audio', variableShape=False, maxShape=None)
            self.countProcessed += 1
        
        if topic == '/irobot_create/sensorPacket':
            
            # Battery state
            batteryVoltage = float(msg.voltage)
            batteryCurrent = float(msg.current)
            batteryCharge = float(msg.batteryCharge) / float(msg.batteryCapacity) * 100.0
            state = np.array([batteryVoltage, batteryCurrent, batteryCharge], dtype=np.float32)
            self.outHdf5.addState('battery', state, t, group='onboard', variableShape=False, maxShape=None)
            
            # Charging state: 3 states (Not charging, Reconditioning/Full charging, Trickle charging/Waiting)
            if msg.chargingState in [0, 5]:
                value = 0
            elif msg.chargingState in [1, 2]:
                value = 1
            else:
                value = 2
            state = np.array([value], dtype=np.uint8)
            self.outHdf5.addState('charge_state', state, t, group='onboard', variableShape=False, maxShape=None)
            
            # Charging source available: 2 states (Internal charger, Homebase)
            state = np.array([msg.internalCharger, msg.homeBase], dtype=np.uint8)
            self.outHdf5.addState('charge_source', state, t, group='onboard', variableShape=False, maxShape=None)
            
            # Collision and cliff detection
            state = np.array([msg.bumpLeft, msg.bumpRight,
                              msg.wheeldropCaster, msg.wheeldropLeft, msg.wheeldropRight,
                              msg.cliffLeft, msg.cliffFrontLeft, msg.cliffFrontRight, msg.cliffRight,
                              msg.wall, msg.virtualWall], dtype=np.uint8)
            self.outHdf5.addState('collision', state, t, group='onboard', variableShape=False, maxShape=None)
            
            self.countProcessed += 1
            
        # Check for ignored message topics
        if lastProcessed == self.countProcessed and topic not in self.ignoredTopics:
            logger.info('Ignoring messages from topic: %s' % (topic))
            self.ignoredTopics.append(topic)
            
        if self.countProcessed > 0 and self.countProcessed % 100 == 0:
            logger.info('Processed %d messages out of %d total messages' % (self.countProcessed, self.countTotal))
        

class SyncStateSaver(StateSaver):

    def __init__(self, outHdf5, topics, queue_size=8, slop=0.5):
        super(SyncStateSaver, self).__init__(outHdf5)
        
        self.topics = topics
        self.slop = rospy.Duration.from_sec(slop)
        self.queue_size = queue_size
        self.queues = [{} for f in fs]

    def add(self, topic, msg, t):
        
        msg_queue = self.queues[self.topics.index(topic)]
        msg_queue[timestamp] = msg
        while len(msg_queue) > self.queue_size:
            del msg_queue[min(msg_queue)]
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                
                avg_timestamp = 0.0
                for t in vv:
                    avg_timestamp += t.to_sec()
                avg_timestamp /= len(vv)
                    
                for topic, msg in msgs:
                    self._process(topic, msg, avg_timestamp)
                    
                for q,t in qt:
                    del q[t]

        self.countTotal += 1

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
                                            compression='gzip', compression_opts=1, shuffle=True)
            self.datasets[dataset.name] = dataset
            self.nbStateSamples[dataset.name] = 0
            
            clockDataset = group.create_dataset('clock',
                                                 shape=(self.chunkSize,), maxshape=(None,), 
                                                 dtype=np.float32)
            self.datasets[clockDataset.name] = clockDataset
            self.nbStateSamples[clockDataset.name] = 0
            
            if variableShape:
                shapeDataset = group.create_dataset('shape',
                                                    shape=(self.chunkSize, state.ndim), maxshape=(None, state.ndim), 
                                                    dtype=np.int32)
                
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
            # rospy.logdebug('Resizing dataset to size %d' % (newShape))
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
            # rospy.logdebug('Closing dataset file: recorded %d state samples' % (self.nbStateSamples))
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
    parser.add_option("-s", "--sync",
                      action="store_false", dest="sync", default=False,
                      help="synchronize messages")
    (options,args) = parser.parse_args(args=args)

    rosBagPath = os.path.abspath(options.input)
    logger.info('Using input rosbag file: %s' % (rosBagPath))

    datasetPath = os.path.abspath(options.output)    
    logger.info('Using output hdf5 file: %s' % (datasetPath))
    
    startTime = float(options.startTime)
    stopTime = float(options.stopTime)
    
    syncTopics = ['/imu/raw',
                  '/irobot_create/sensorPacket',
                  '/video/left/raw/compressed',
                  '/video/right/raw/compressed',
                  '/audio/left/raw',
                  '/audio/right/raw']
    
    with Hdf5Bag(datasetPath, chunkSize=32, overwrite=True) as outHdf5:
        
        baseTime = None
        with rosbag.Bag(rosBagPath) as inbag:
            
            if options.sync:
                saver = SyncStateSaver(outHdf5)
            else:
                saver = StateSaver(outHdf5)
            
            for topic, msg, timestamp in inbag.read_messages():
                
                # Make clock time relative to the first timestamp
                if baseTime is None:
                    baseTime = timestamp.to_sec()
                t = timestamp.to_sec() - baseTime
                assert t >= 0.0
                
                # Validate if within timerange
                if t < startTime:
                    continue
                
                saver.add(topic, msg, t)
                
                if stopTime >= 0.0 and t > stopTime:
                    logger.info('Stop time (%f sec) reached' % (stopTime))
                    break
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    