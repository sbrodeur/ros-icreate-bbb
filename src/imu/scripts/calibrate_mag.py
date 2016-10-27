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
from optparse import OptionParser
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import rospy
from imu.msg import MagneticFieldBatch

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

def PCA(x):
    # Find the eigenvectors of the covariance matrix
    m = np.mean(x, axis=0)
    x = x - m
    [_, V] = np.linalg.eig(np.cov(x.T))
    return V

def calculateBankAngleZ(data):
    # Find the normal vector to the data that points upward (positive z)
    _, n = fitHyperplane(data)
    if n[2] < 0:
        n = -n
    
    # Find the rotation matrix between the reference and data vectors
    vref = np.array([0.0, 0.0, 1.0])
    vdata = n
    R = rotationMatrix(vdata, vref)
    return R

def calculateSoftIronXY(data):
    # NOTE: ignore z axis
    
    # Compute rotation matrix for the ellipse
    V = PCA(data[:,:2])
    Rxy = np.identity(3)
    Rxy[:2,:2] = V
    
    # Apply transformation
    newdata = np.dot(Rxy, data.T).T
    
    # Compute scaling factors for x and y axis
    xscale = np.max(newdata[:,0], axis=0) - np.min(newdata[:,0], axis=0)
    yscale = np.max(newdata[:,1], axis=0) - np.min(newdata[:,1], axis=0)
    scale = np.mean([xscale, yscale])
    factorx = scale / xscale
    factory = scale / yscale
    
    return Rxy, factorx, factory

class DataRecorder:
    
    def __init__(self):
        self.data = []
  
        input = rospy.get_param('~input', '/imu/mag')
        rospy.Subscriber(input, MagneticFieldBatch, self.callback)
  
    def callback(self, msg):
        vector = np.array([msg.magnetic_fields[-1].x, msg.magnetic_fields[-1].y, msg.magnetic_fields[-1].z], dtype=np.float64)
        self.data.append(vector)
        
    def spin(self, timeout=None):
        
        startTime = time.time()
        try:
            rospy.init_node('calibrate_mag', log_level=rospy.INFO)
            
            rospy.loginfo('Recording started')
            while not rospy.is_shutdown():
                elapsed = time.time() - startTime
                if timeout is not None and elapsed > timeout:
                    break
                time.sleep(0.25)
    
        except rospy.ROSInterruptException: pass
        except KeyboardInterrupt: pass
        
        rospy.loginfo('Recording stopped')
        
    def getData(self):
        return np.array(self.data)

def main(args=None):

    parser = OptionParser()
    parser.add_option("-i", "--input", dest="input", default=None,
                      help='specify the path of the input cvs file')
    parser.add_option("-o", "--output", dest="output", default=None,
                      help='specify the path of the output cvs file')
    parser.add_option("-d", "--display-data",
                      action="store_true", dest="displayData", default=False,
                      help="Display the raw and converted data")
    (options,args) = parser.parse_args(args=args)

    if options.input == None:
        recorder = DataRecorder()
        recorder.spin()
        data = recorder.getData()

        outputCvsPath = os.path.abspath(options.output)
        logger.info('Using output cvs file: %s' % (outputCvsPath))
        np.savetxt(outputCvsPath, data, delimiter=",")
    else:
        # Load data from the cvs file
        inputCvsPath = os.path.abspath(options.input)
        logger.info('Using input cvs file: %s' % (inputCvsPath))
        with open(inputCvsPath,"r") as f:
            data = np.loadtxt(f, delimiter=",")
    
    rospy.loginfo('Number of input data points: %d' % (data.shape[0]))
    
    # Compute the offset (based on median)
    offset = np.min(data, axis=0, keepdims=True) + 0.5 * (np.max(data, axis=0, keepdims=True) -
                                                          np.min(data, axis=0, keepdims=True))
    data -= offset
    
    Rz = calculateBankAngleZ(data)
    Rxy, factorx, factory = calculateSoftIronXY(data)
    
    # Convert data (z axis)
    newdata = np.dot(Rz, data.T).T
    
    # Convert data (x-y plane)
    # Apply the rotation, rescale, then apply the inverse rotation (transposed of the rotation matrix).
    newdata = np.dot(Rxy, newdata.T).T
    newdata[:,0] *= factorx
    newdata[:,1] *= factory
    newdata = np.dot(Rxy.T, newdata.T).T
    factors = np.array([factorx, factory])
    
    # Calculate ranges
    xmin, ymin, zmin = np.min(newdata, axis=0)
    xmax, ymax, zmax = np.max(newdata, axis=0)
    rospy.loginfo('X scale: %f, %f (range of %f)' % (xmin, xmax, xmax - xmin))
    rospy.loginfo('Y scale: %f, %f (range of %f)' % (ymin, ymax, ymax - ymin))
    rospy.loginfo('Z scale: %f, %f (range of %f)' % (zmin, zmax, zmax - zmin))
    limits = [0.9*min(xmin, ymin), 1.1*max(xmax, ymax)]
    
    rospy.loginfo('Use this offset vector to correct for offset (x, y, x axes): \n ' + str(np.squeeze(offset)))
    rospy.loginfo('Use this rotation matrix to correct for bank angle (z axis): \n ' + str(Rz))
    rospy.loginfo('Use this rotation matrix to correct for soft-iron effects (x and y axis): \n ' + str(Rxy))
    rospy.loginfo('Use these factors to correct for soft-iron effects (x-y plane): \n ' + str(factors))
    
    if options.displayData:
        fig = plt.figure(figsize=(10,10),facecolor='white')
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.view_init(elev=90.0, azim=0.0)
        ax.set_xlim(limits)
        ax.set_ylim(limits)
        
        # Raw
        xs, ys, zs = data[:,0], data[:,1], data[:,2]
        ax.scatter(xs, ys, zs, c='b', marker='o', label=['raw'])
    
        # Converted
        xs, ys, zs = newdata[:,0], newdata[:,1], newdata[:,2]
        ax.scatter(xs, ys, zs, c='r', marker='o', label=['corrected'])
        
        ax.legend()
        plt.show()
        
    rospy.loginfo('All done.')

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    
