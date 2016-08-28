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
from audio.msg import AudioPacket
   
class RealtimePlotter:

    def __init__(self):
       
        self.q = Queue.Queue(10)

        pylab.ion()
        fig = pylab.figure(1)
        ax = fig.add_subplot(111)
        ax.grid(True)
        ax.set_title("Realtime Audio Waveform Plot")
        ax.set_xlabel("Time")
        ax.set_ylabel("Amplitude")
        
        self.fig =fig
        self.ax = ax
        self.line1=None
        
        input = rospy.get_param('~input', '/audio/default/raw')
        rospy.Subscriber(input, AudioPacket, RealtimePlotter.callback, self)

    def animate(self):
        try:
            data = self.q.get(True, 0.25)
            
            size = len(data)
            xdata=pylab.arange(0,size,1)
            ydata=pylab.array(data)
             
            if self.line1 == None:
                # Create line plot
                self.line1 = self.ax.plot(xdata,ydata,'-')
            else:
                # Update existing line plot
                self.line1[0].set_data(xdata,ydata)
            
            self.ax.axis([xdata.min(),xdata.max(),-1.1,1.1])
            self.fig.canvas.draw() 
        except Queue.Empty:
            pass
        
    @staticmethod
    def callback(data, self):
        if not self.q.full():
            # Convert to numpy array and normalize in range [-1,1]
            audio = np.array(data.data)
            audio = audio / 32767.0
            self.q.put(audio)
        else:
            rospy.logwarn('Audio queue is full!')
        
if __name__ == '__main__':
    
    try:
        rospy.init_node('viewer')
        plt = RealtimePlotter()
        
        while not rospy.is_shutdown():
            plt.animate()

    except rospy.ROSInterruptException: pass

