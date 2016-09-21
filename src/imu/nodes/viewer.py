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
from sensor_msgs.msg import Imu

class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def set_data(self, xs, ys, zs):
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)
        
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
        
class RealtimeOrientationPlotter:

    def __init__(self):
       
        self.q = Queue.Queue(1)

        pylab.ion()
        fig = pylab.figure(facecolor='white')
        ax = fig.gca(projection='3d')
        ax.grid(True)
        ax.set_title("Realtime Orientation Plot")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.view_init(elev=45.0, azim=45.0)
        
        self.fig = fig
        self.ax = ax
        self.arrows = []
        
        input = rospy.get_param('~input', '/imu/data')
        rospy.Subscriber(input, Imu, RealtimeOrientationPlotter.callback, self)

    def animate(self):
        try:
            quaternion = self.q.get(True, 0.25)
            
            # Convert from quaternion (w, x, y, z) to rotation matrix
            R = quat2mat(quaternion)
            
            # Apply the rotation to the axis vectors (pointing in Y-axis)
            directions = np.eye(3) # x, y, z as column vectors
            vectors = np.dot(R, directions)
            assert np.allclose(np.linalg.norm(vectors, 2, axis=0), np.ones((3,)), atol=1e-6)
            
            if len(self.arrows) == 0:
                # Create line plot
                labels = ['x', 'y', 'z']
                colors = ['r', 'g', 'b']
                for i in range(3):
                    x,y,z = vectors[:,i]
                    arrow = Arrow3D([0.0, x], [0.0, y], [0.0, z], mutation_scale=20, 
                                    lw=3, arrowstyle="-|>", color=colors[i], label=labels[i])
                    self.arrows.append(arrow)
                    self.ax.add_artist(arrow)
                
                proxies = [pylab.Rectangle((0, 0), 1, 1, fc=c) for c in colors]
                legend = self.ax.legend(proxies, labels, loc='upper right')
            else:
                # Update existing line plot
                for i in range(3):
                    x,y,z = vectors[:,i]
                    self.arrows[i].set_data([0.0, x], [0.0, y], [0.0, z])
            
            self.ax.axis([-1.0, 1.0, -1.0, 1.0])
            self.fig.canvas.draw()
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(msg, self):
        if not self.q.full():
            # Orientation (as quaternion [w, x, y, z])
            quaternion = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z], dtype=np.float64)
            self.q.put(quaternion)
            
        else:
            # Discard the message
            pass
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer')
        plt = RealtimeOrientationPlotter()
         
        while not rospy.is_shutdown():
            plt.animate()
 
    except rospy.ROSInterruptException: pass
