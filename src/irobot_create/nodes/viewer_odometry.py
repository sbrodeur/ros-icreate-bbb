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
import pylab
import numpy
import Queue
import numpy as np

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import rospy
from nav_msgs.msg import Odometry
        
# Adapted from: https://afni.nimh.nih.gov/pub/dist/src/pkundu/meica.libs/nibabel/quaternions.py
def quat2mat(q):
    w, x, y, z = q
    Nq = w*w + x*x + y*y + z*z
    if Nq < np.finfo(np.float64).eps:
        return np.eye(3)
    s = 2.0/Nq
    X = x*s
    Y = y*s
    Z = z*s
    wX = w*X; wY = w*Y; wZ = w*Z
    xX = x*X; xY = x*Y; xZ = x*Z
    yY = y*Y; yZ = y*Z; zZ = z*Z
    return np.array(
           [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
            [ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
            [ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])
        
class RealtimePositionPlotter:

    def __init__(self, windowSize=1000, downsampleRatio=1):
        self.windowSize = windowSize
        self.downsampleRatio = downsampleRatio
        self.q = Queue.Queue(1)
        self.nbPoints = 0
        self.n = 0
        self.data = np.zeros((windowSize, 2), dtype=np.float32)

        pylab.ion()

        fig = plt.figure(figsize=(5,4), facecolor='white', frameon=False)
        ax = fig.add_subplot(111)
        
        self.scatPast = ax.scatter([0,], [0,], s=10)
        self.scatCur = ax.scatter([0,], [0,], c=[1.0,0.0,0.0], s=50)
        fig.tight_layout()
        fig.subplots_adjust(left=0.20, bottom=0.15)
        
        ax.grid(True)
        ax.set_title('Odometry')
        ax.set_xlabel("x [meter]")
        ax.set_ylabel("y [meter]")
        ax.axis([-1.0, 1.0, -1.0, 1.0])
        
        self.fig = fig
        self.ax = ax
        self.arrows = []
        
        input = rospy.get_param('~input', '/irobot_create/odom')
        rospy.Subscriber(input, Odometry, RealtimePositionPlotter.callback, self)

    def animate(self):
        try:
            (position, orientation) = self.q.get(True, 0.25)

            # Update buffer
            self.data[0:-1:,:] = self.data[1::,:]
            self.data[-1,:] = position[0:2]
            if self.nbPoints < self.windowSize:
                self.nbPoints += 1
                
            # Convert from quaternion (w, x, y, z) to rotation matrix
            R = quat2mat(orientation)
            
            # Apply the rotation to the vector pointing in the forward direction (x-axis)
            direction = np.array([1.0, 0.0, 0.0])
            vector = np.dot(R, direction)
            vector /= np.linalg.norm(vector, 2)
            
            if self.n % int(self.downsampleRatio) == 0:
                cdata = self.data[self.windowSize-self.nbPoints:,:]
                self.scatPast.set_offsets(cdata)
                self.scatCur.set_offsets(cdata[-1,:])
                
                border = 0.5
                xlim = np.array([np.min(cdata[:,0])-border, np.max(cdata[:,0])+border])
                ylim = np.array([np.min(cdata[:,1])-border, np.max(cdata[:,1])+border])
                scale = np.max([xlim[1] - xlim[0], ylim[1] - ylim[0]])
                self.ax.set_xlim(xlim)
                self.ax.set_ylim(ylim)
    
                x, y = cdata[-1,0], cdata[-1,1]
                dx, dy = vector[0] * 0.05 * scale, vector[1] * 0.05 * scale
                lines = self.ax.plot([x, x+dx], [y, y+dy], c='r')
                
                self.fig.canvas.draw()
                
                lines.pop(0).remove()
            
            self.n += 1
            
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(msg, self):
        if not self.q.full():
            # Position (as vector [x, y, z]) and orientation (as quaternion [w, x, y, z])
            position = np.array([msg.pose.pose.position.x,
                                 msg.pose.pose.position.y,
                                 msg.pose.pose.position.z], dtype=np.float64)
            orientation = np.array([msg.pose.pose.orientation.w, 
                                    msg.pose.pose.orientation.x, 
                                    msg.pose.pose.orientation.y, 
                                    msg.pose.pose.orientation.z], dtype=np.float64)
            self.q.put((position, orientation))
            
        else:
            # Discard the message
            pass
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer_odometry')
        plt = RealtimePositionPlotter()
         
        while not rospy.is_shutdown():
            plt.animate()
 
    except rospy.ROSInterruptException: pass
