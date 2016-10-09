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
import numpy as np

from h5utils import Hdf5Dataset
from optparse import OptionParser

logger = logging.getLogger(__name__)

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", type='string', default=None,
                      help='specify the path of the input HDF5 file')
    parser.add_option("-o", "--output", dest="output", type='string', default=None,
                      help='specify the path of the output HDF5 file')
    (options,args) = parser.parse_args(args=args)

    inputDatasetPath = os.path.abspath(options.input)
    logger.info('Using input HDF5 dataset file: %s' % (inputDatasetPath))

    outputDatasetPath = os.path.abspath(options.output)    
    logger.info('Using output HDF5 dataset file: %s' % (outputDatasetPath))

    with Hdf5Dataset(outputDatasetPath, mode='w') as outHdf5:
        with Hdf5Dataset(inputDatasetPath, mode='r') as inHdf5:
            
            # Process states
            for state in inHdf5.getAllStates():
                name, group, raw, clock, shape = state
                
                # Sort by time
                indices = np.argsort(clock)
                clock = clock[indices]
                raw = raw[indices]
                if shape is not None:
                    shape = shape[indices]
                if not np.allclose(indices, np.arange(len(clock), dtype=indices.dtype)):
                    logger.info('%s had unsorted times' % (ngroup + '/' + name))
                
                # Write to output file
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
