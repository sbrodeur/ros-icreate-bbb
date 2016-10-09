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

logger = logging.getLogger(__name__)

class Hdf5Dataset:

    def __init__(self, filePath, mode='r', chunkSize=32):
        self.chunkSize = chunkSize
        
        # Possible optimization: libver='latest'
        self.h = h5py.File(filePath, mode)
        
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


    def addStates(self, name, raw, clock, group=None, shape=None):
        
        if group is not None:
            baseName = '/' + group + '/' + name
        else:
            baseName = '/' + name
        
        datasetName = baseName + '/raw'
        clockName = baseName + '/clock'
        shapeName = baseName + '/shape'
        
        assert datasetName not in self.datasets
        
        if group is not None:
            try:
                file = self.h.create_group(group)
            except ValueError:
                file = self.h[group]
        else:
            file = self.h
    
        group = file.create_group(name)
    
        if shape is not None:
            dataShape = raw.shape[1]
        else:
            dataShape = raw.shape
        dataset = group.create_dataset('raw', data=raw,
                                        chunks=True,
                                        compression='gzip', compression_opts=6, shuffle=True)
        self.datasets[dataset.name] = dataset
        
        clockDataset = group.create_dataset('clock', data=clock,
                                            chunks=True,
                                            compression='gzip', compression_opts=6, shuffle=True,
                                            dtype=np.float64)
        self.datasets[clockDataset.name] = clockDataset
        
        if shape is not None:
            shapeDataset = group.create_dataset('shape', data=shape,
                                                chunks=True,
                                                compression='gzip', compression_opts=6, shuffle=True,
                                                dtype=np.int64)
                
            self.datasets[shapeDataset.name] = shapeDataset

    def getAllClocks(self):
        
        # Loop over all groups at the root level
        for key in self.h.keys():
            group = self.h[key]
            if 'clock' in group.keys():
                clock = np.array(group['clock'])
                yield [key, None, clock]
            else:
                # Loop for all group elements
                for subkey in group.keys():
                    subgroup = group[subkey]
                    assert 'clock' in subgroup.keys()
                
                    clock = np.array(subgroup['clock'])
                    yield [subkey, key, clock]

    def getAllStates(self):
        
        # Loop over all groups at the root level
        for key in self.h.keys():
            group = self.h[key]
            if 'raw' in group.keys():
                raw = np.array(group['raw'])
                clock = np.array(group['clock'])
                if 'shape' in group.keys():
                    shape = np.array(group['shape'])
                else:
                    shape = None
                    
                state = [key, None, raw, clock, shape]
                yield state
            else:
                # Loop for all group elements
                for subkey in group.keys():
                    subgroup = group[subkey]
                    assert 'raw' in subgroup.keys()
                
                    raw = np.array(subgroup['raw'])
                    clock = np.array(subgroup['clock'])
                    if 'shape' in subgroup.keys():
                        shape = np.array(subgroup['shape'])
                    else:
                        shape = None
                
                    state = [subkey, key, raw, clock, shape]
                    yield state
            
    def close(self):
        if len(self.nbStateSamples) > 0:
            for name, dataset in self.datasets.iteritems():
                # Truncate dataset to the actual number of state samples
                dataset.resize(self.nbStateSamples[name], axis=0)
        self.h.close()

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self.close()
