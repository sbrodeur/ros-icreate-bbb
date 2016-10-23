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

import rospy
from sensor_msgs.msg import Imu
from imu.msg import ImuBatch

class RealtimeImuPlotter:

    def __init__(self, windowSize=512):

        self.windowSize = windowSize
        self.data = np.zeros((windowSize, 6))
        
        self.q = Queue.Queue(10)

        pylab.ion()
        fig = pylab.figure(1)
        ax1 = fig.add_subplot(211)
        ax1.grid(True)
        ax1.set_title("Realtime Accelerometer Plot")
        ax1.set_xlabel("Time")
        ax1.set_ylabel("Linear acceleration [m/s^2]")

        ax2 = fig.add_subplot(212)
        ax2.grid(True)
        ax2.set_title("Realtime Gyroscope Plot")
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Angular velocity [rad/sec]")

        self.fig = fig
        self.axes = [ax1, ax2]
        
        xdata=pylab.arange(0,self.windowSize,1)
        ydata=pylab.zeros(self.windowSize)
        self.lines = []
        self.lines.append(ax1.plot(xdata,ydata,'-r'))
        self.lines.append(ax1.plot(xdata,ydata,'-g'))
        self.lines.append(ax1.plot(xdata,ydata,'-b'))
        self.lines.append(ax2.plot(xdata,ydata,'-r'))
        self.lines.append(ax2.plot(xdata,ydata,'-g'))
        self.lines.append(ax2.plot(xdata,ydata,'-b'))
        
        ax1.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left')
        ax2.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left')
        
        input = rospy.get_param('~input', '/imu/data_raw')
        batch = rospy.get_param('~batch', False)
        
        if batch:
            rospy.Subscriber(input, ImuBatch, RealtimeImuPlotter.callback, self)
        else:
            rospy.Subscriber(input, Imu, RealtimeImuPlotter.callback, self)

    def animate(self):
        try:
            data = self.q.get(True, 0.25)
            xdata=pylab.arange(0,self.windowSize,1)
            for i in range(0, 6):
                for s in range(data.shape[0]):
                    self.data[0:-1:,i] = self.data[1::,i]
                    self.data[-1,i] = data[s, i]
                    
                ydata=pylab.array(self.data[:,i])
             
                # Update existing line plots
                self.lines[i][0].set_data(xdata,ydata)
            
            self.axes[0].axis([xdata.min(),xdata.max(),np.min(self.data[:,:3]),np.max(self.data[:,:3])])
            self.axes[1].axis([xdata.min(),xdata.max(),np.min(self.data[:,3:]),np.max(self.data[:,3:])])
            
            self.fig.canvas.draw()
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(msg, self):
        if not self.q.full():
            
            if isinstance(msg, Imu):
                data = np.array([[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                                  msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]]);
                self.q.put(data)
            elif isinstance(msg, ImuBatch):
                nbSamples = len(msg.linear_accelerations);
                data = []
                for i in range(nbSamples):
                    dataSample = np.array([msg.linear_accelerations[i].x, msg.linear_accelerations[i].y, msg.linear_accelerations[i].z,
                                      msg.angular_velocities[i].x, msg.angular_velocities[i].y, msg.angular_velocities[i].z]);
                    data.append(dataSample)
                data = np.array(data)
                self.q.put(data)
        else:
            pass
            #rospy.logwarn('Plotting queue is full!')
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer')
        plt = RealtimeImuPlotter()
        
        while not rospy.is_shutdown():
            plt.animate()

    except rospy.ROSInterruptException: pass

