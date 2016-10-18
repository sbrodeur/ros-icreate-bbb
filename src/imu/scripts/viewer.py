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
        ax1.set_ylabel("Amplitude")
        ax1.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left')

        ax2 = fig.add_subplot(212)
        ax2.grid(True)
        ax2.set_title("Realtime Gyroscope Plot")
        ax2.set_xlabel("Time")
        ax2.set_ylabel("Amplitude")
        ax2.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left')

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
        
        input = rospy.get_param('~input', '/imu/data_raw')
        rospy.Subscriber(input, Imu, RealtimeImuPlotter.callback, self)

    def animate(self):
        try:
            data = self.q.get(True, 0.25)
            xdata=pylab.arange(0,self.windowSize,1)
            for i in range(0, 6):
                self.data[0:-1:,i] = self.data[1::,i]
                self.data[-1,i] = data[0, i]
                ydata=pylab.array(self.data[:,i])
             
                # Update existing line plots
                self.lines[i][0].set_data(xdata,ydata)
                self.axes[i/3].axis([xdata.min(),xdata.max(),ydata.min(),ydata.max()])
            
            self.fig.canvas.draw() 
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(data, self):
        if not self.q.full():
            data = np.array([[data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z,
                              data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]]);
            self.q.put(data)
        else:
            rospy.logwarn('Plotting queue is full!')
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer')
        plt = RealtimeImuPlotter()
        
        while not rospy.is_shutdown():
            plt.animate()

    except rospy.ROSInterruptException: pass

