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
import numpy
import Queue
import numpy as np
import csv

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import rospy
from sensor_msgs.msg import MagneticField
from imu.msg import MagneticFieldBatch

# Adapted from: http://stackoverflow.com/questions/22867620/putting-arrowheads-on-vectors-in-matplotlibs-3d-plot
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
        
class RealtimeMagneticFieldPlotter:

    def __init__(self):
       
        self.q = Queue.Queue(1)

        plt.ion()
        fig = plt.figure(facecolor='white')
        ax = fig.gca(projection='3d')
        ax.grid(True)
        ax.set_title("Realtime Magnetic Field Plot")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.view_init(elev=45.0, azim=45.0)
        
        self.fig = fig
        self.ax = ax
        self.arrow = None
        self.text = None
        
        
        input = rospy.get_param('~input', '/imu/mag')
        batch = rospy.get_param('~batch', False)
        
        if batch:
            rospy.Subscriber(input, MagneticFieldBatch, RealtimeMagneticFieldPlotter.callback, self)
        else:
            rospy.Subscriber(input, MagneticField, RealtimeMagneticFieldPlotter.callback, self)
        
        self.genMagProfile = rospy.get_param('~generate_mag_profile', False)
        if(self.genMagProfile):
            self.csvfile = open('magPoints.csv','w')
            self.magPointWriter = csv.writer(self.csvfile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)


    def animate(self):
        try:
            vector = self.q.get(True, 0.25)
            x,y,z = vector
            
            if self.arrow is None:
                # Create line plot
                arrow = Arrow3D([0.0, x], [0.0, y], [0.0, z], mutation_scale=20, 
                                lw=3, arrowstyle="-|>", color='r')
                self.arrow = arrow
                self.ax.add_artist(arrow)
            else:
                # Update existing line plot
                self.arrow.set_data([0.0, x], [0.0, y], [0.0, z])
                
            limits = [-1.0e-4, 1.0e-4]
            norm = np.linalg.norm(vector, 2) * 1e6
            if self.text is None:
                text = self.ax.text(0, 0, limits[1]/5.0, "%4.1f uT" % (norm),
                                    horizontalalignment='center',
                                    verticalalignment='top')
                self.text = text
            else:
                self.text.set_text("%4.1f uT" % (norm))
                
            
            self.ax.set_xlim3d(limits)
            self.ax.set_ylim3d(limits)
            self.ax.set_zlim3d(limits)
            self.fig.canvas.draw()
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(msg, self):
        if not self.q.full():
            
            if isinstance(msg, MagneticField):
                # Magnetometer (as 3D vector [x, y, z])
                vector = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z], dtype=np.float64)
                self.q.put(vector)
            elif isinstance(msg, MagneticFieldBatch):
                # Magnetometer in batch mode (as 3D vector [x, y, z])
                # NOTE: only show the last sample in the batch
                vector = np.array([msg.magnetic_fields[-1].x, msg.magnetic_fields[-1].y, msg.magnetic_fields[-1].z], dtype=np.float64)
                self.q.put(vector)
            
            if(self.genMagProfile):
                self.magPointWriter.writerow([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
            
        else:
            # Discard the message
            pass
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer')
        viewer = RealtimeMagneticFieldPlotter()
         
        while not rospy.is_shutdown():
            viewer.animate()
 
    except rospy.ROSInterruptException:
        if(self.genMagProfile):
            self.csvfile.close()
        pass
