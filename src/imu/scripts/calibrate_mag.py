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
from optparse import OptionParser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)

# Adapted from: https://gist.github.com/lambdalisue/7201028
def fitHyperplane(X):
    # Find the average of points (centroid) along the columns
    C = np.average(X, axis=0)
    # Create CX vector (centroid to point) matrix
    CX = X - C
    # Singular value decomposition
    U, S, V = np.linalg.svd(CX)
    # The last row of V matrix indicate the eigenvectors of
    # smallest eigenvalues (singular values).
    N = V[-1]
    return C, N

# See: http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
def rotationMatrix(a, b):
    """
    Compute the rotation matrix to transform vector a into b
    """
    
    # Normalize the two vectors
    a /= np.linalg.norm(a, 2)
    b /= np.linalg.norm(b, 2)
    
    # Compute the rotation matrix
    v = np.cross(a, b)
    s = np.linalg.norm(v, 2)
    c = np.dot(a, b)
    vx = np.array([[0,    -v[2], v[1]],
                   [v[2],    0, -v[0]],
                   [-v[1], v[0],   0]])
    
    R = np.identity(3) + vx + np.square(vx)*(1.0/(1.0 + c))
    return R


def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input cvs file')
    parser.add_option("-d", "--display-data",
                      action="store_true", dest="displayData", default=False,
                      help="Display the raw and converted data")
    (options,args) = parser.parse_args(args=args)

    cvsPath = os.path.abspath(options.input)
    logger.info('Using input cvs file: %s' % (cvsPath))

    # Load data from the cvs file
    data = np.loadtxt(open(cvsPath,"r"), delimiter=",")
    
    # Find the normal vector to the data that points upward (positive z)
    _, n = fitHyperplane(data)
    if n[2] < 0:
        n = -n
    
    # Find the rotation matrix between the reference and data vectors
    vref = np.array([0.0, 0.0, 1.0])
    vdata = n
    R = rotationMatrix(vdata, vref)
    
    # Convert data
    newdata = np.dot(R, data.T).T
    
    # Calculate ranges
    xmin, ymin, zmin = np.min(newdata, axis=0)
    xmax, ymax, zmax = np.max(newdata, axis=0)
    logger.info('X scale: %f, %f (range of %f)' % (xmin, xmax, xmax - xmin))
    logger.info('Y scale: %f, %f (range of %f)' % (ymin, ymax, ymax - ymin))
    logger.info('Z scale: %f, %f (range of %f)' % (zmin, zmax, zmax - zmin))
    
    logger.info('Use this rotation matrix: \n ' + str(R))
    
    if options.displayData:
        fig = plt.figure(facecolor='white')
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        
        # Raw
        xs, ys, zs = data[:,0], data[:,1], data[:,2]
        ax.scatter(xs, ys, zs, c='b', marker='o', label=['raw'])
    
        # Converted
        xs, ys, zs = newdata[:,0], newdata[:,1], newdata[:,2]
        ax.scatter(xs, ys, zs, c='r', marker='o', label=['corrected'])
        
        ax.legend()
        plt.show()
        
    logger.info('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    