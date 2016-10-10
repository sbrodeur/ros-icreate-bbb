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
import scipy
import numpy as np

from h5utils import Hdf5Dataset
from optparse import OptionParser
from scipy.interpolate import interp1d

logger = logging.getLogger(__name__)

def estimate_audio_timestamps(raw, clock, fs):
    
    logger.info('Estimating audio timestamps for chunk flattening')
    
    packetSize = raw.shape[1]
    t_sample = 1.0/fs
    
    # Estimate per-sample timestamp
    clocks = []           
    for t in clock:
        # NOTE: Ajust timestamps so that they are relative to the true capture time
        t_packet = t + np.arange(packetSize)*t_sample - packetSize*t_sample
        
        if len(clocks) > 0:
            t_packet_start = t_packet[0]
            t_last_packet_end = clocks[-1][-1]
            
            if t_packet_start <= t_last_packet_end:
                # Packet is timestamped too early, which mean we may have got it from the buffer
                # and it is thus sequential to the last packet
                t_packet = t_last_packet_end + np.arange(packetSize)*t_sample + t_sample
        
        clocks.append(t_packet)
     
    # Concatenate all audio chunks and timestamps
    raw = np.ravel(raw)
    clock = np.concatenate(clocks)

    max_t_diff = np.max(clock[1:] - clock[:-1])
    logger.info('Maximum time difference found between audio packet: %f sec' % (max_t_diff))
    
    sync_clock = np.linspace(np.min(clock), np.max(clock),
                             num=int((np.max(clock) - np.min(clock)) * fs))

    # Interpolation
    f = interp1d(clock, raw, kind='nearest', axis=0, copy=False)
    sync_raw = f(sync_clock).astype(raw.dtype)

    return sync_raw, sync_clock

def synchronize_audio(raw, clock, sync_clock):
    
    logger.info('Chunking audio data to synchronization clock')
    
    fs = 1.0/np.mean(clock[1:] - clock[:-1])
    sync_fs = 1.0/np.mean(sync_clock[1:] - sync_clock[:-1])
    chunk_size = int(fs/sync_fs)
    logger.info('Estimated sampling frequency: raw = %f Hz, sync = %f Hz' % (fs, sync_fs))
    
    # Interpolation using indices
    f = interp1d(clock, np.arange(raw.shape[0]), kind='nearest', axis=0, copy=False)
    indices = f(sync_clock).astype(np.int32)
    
    raw_chunks = []
    clock = []
    for idx, t in zip(indices,sync_clock):
        
        if chunk_size % 2 == 0:
            startIdx = idx - chunk_size/2
            stopIdx = idx + chunk_size/2 - 1
        else:
            startIdx = idx - chunk_size/2
            stopIdx = idx + chunk_size/2
        
        if startIdx >= 0 and stopIdx+1 <= raw.shape[0]:
            raw_chunks.append(raw[startIdx:stopIdx+1])
            
        elif startIdx < 0:
            raw_chunk = np.concatenate((np.zeros(np.abs(startIdx)),raw[:stopIdx+1]))
            raw_chunks.append(raw_chunk)
            needed_border = abs(startIdx) * fs
            logger.warn('Could only extract partial audio chunk at time %f sec (beginning padded with zeros): consider adding a border of at least %f sec' % (t, needed_border))
    
        elif stopIdx+1 > raw.shape[0]:
            raw_chunk = np.concatenate((raw[:stopIdx+1], np.zeros(stopIdx+1 - raw.shape[0])))
            raw_chunks.append(raw_chunk)
            needed_border = abs(stopIdx+1 - raw.shape[0]) * fs
            logger.warn('Could only extract partial audio chunk at time %f sec (end padded with zeros): consider adding a border of at least %f sec' % (t, needed_border))
    
        else:
            raise Exception('Could not extract audio chunk at time %f sec: no data available around this timestamp' % (t))
            
        clock.append(t)
    
    raw_chunks = np.vstack(raw_chunks)
    clock = np.array(clock)
    
    return raw_chunks, clock

def synchronize_clocks(hdf5bag, fs=20.0, interpolation='nearest', safeBorder=0.0):
    
    validStartTime = np.max([clock[0] + safeBorder for _,_,clock in hdf5bag.getAllClocks()])
    validStopTime = np.min([clock[-1] - safeBorder for _,_,clock in hdf5bag.getAllClocks()])
    sync_clock = np.linspace(validStartTime, validStopTime,
                             num=int((validStopTime - validStartTime) * fs))
    logger.debug('Found valid clock range: [%f, %f] sec' % (validStartTime, validStopTime))
    
    for state in hdf5bag.getAllStates():
        name, group, raw, clock, shape = state
        logger.debug('Interpolating state %s (group: %s)' % (name, str(group)))

        if group == 'audio' and interpolation == 'nearest':
            # Special processing for audio because we have to rechunk the data
            raw, clock = estimate_audio_timestamps(raw, clock, fs=16000)
            sync_raw, sync_clock = synchronize_audio(raw, clock, sync_clock)
            sync_shape = None
        
        elif interpolation == 'nearest':
            # Interpolation using indices for reduced memory usage
            indices = np.arange(raw.shape[0], dtype=np.int)
            f = interp1d(clock, indices, kind='nearest', axis=0, copy=False)
            sync_indices = f(sync_clock).astype(indices.dtype)
            sync_raw = raw[sync_indices]
            
            if shape is not None:
                sync_shape = shape[sync_indices]
            else:
                sync_shape = None
        else:
            raise Exception('Unsupported interpolation: %s' % (interpolation))
            
        state = [name, group, sync_raw, sync_clock, sync_shape]
        yield state

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", type='string', default=None,
                      help='specify the path of the input HDF5 file')
    parser.add_option("-o", "--output", dest="output", type='string', default=None,
                      help='specify the path of the output HDF5 file')
    parser.add_option("-p", "--interpolation", dest="interpolation", type='string', default='nearest',
                      help='specify the path of the output HDF5 file')
    parser.add_option("-f", "--fs", dest="fs", type='float', default=20.0,
                      help='specify the output sampling frequency')
    parser.add_option("-b", "--border", dest="border", type='float', default=0.0,
                      help='specify the border to keep')
    (options,args) = parser.parse_args(args=args)

    inputDatasetPath = os.path.abspath(options.input)
    logger.info('Using input HDF5 dataset file: %s' % (inputDatasetPath))

    outputDatasetPath = os.path.abspath(options.output)    
    logger.info('Using output HDF5 dataset file: %s' % (outputDatasetPath))

    datasets = dict()
    with Hdf5Dataset(outputDatasetPath, mode='w') as outHdf5:
        with Hdf5Dataset(inputDatasetPath, mode='r') as inHdf5:
            
            # Print information
            for state in inHdf5.getAllClocks():
                name, group, clock = state
                if group is not None:
                    ngroup = '/' + group
                else:
                    ngroup = ''
                fs = 1.0/np.mean(clock[1:] - clock[:-1])
                logger.info('Found dataset: %s, fs=%f Hz' % (ngroup + '/' + name, fs))
                
            # Process states
            for state in synchronize_clocks(inHdf5, options.fs, options.interpolation, options.border):
                name, group, raw, clock, shape = state
                
                if group is not None:
                    ngroup = '/' + group
                else:
                    ngroup = ''
                fs = 1.0/np.mean(clock[1:] - clock[:-1])
                logger.info('Writing to output HDF5 dataset file: %s, shape=%s, fs=%f Hz' % (ngroup + '/' + name, str(raw.shape), fs))
                outHdf5.addStates(name, raw, clock, group, shape)
                        
    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
