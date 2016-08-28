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
import random
import sys
import itertools
import wx

import matplotlib
matplotlib.use('WXAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_wxagg import \
    FigureCanvasWxAgg as FigCanvas, \
    NavigationToolbar2WxAgg as NavigationToolbar
import numpy as np
from numpy import random
import pylab
import threading

import rospy
import message_filters
from message_filters import TimeSynchronizer
from sensor_msgs.msg import Image
from std_msgs.msg import MultiArrayLayout, MultiArrayDimension
from imu.msg import ImuPacket
from audio.msg import AudioPacket
from irobot_create.msg import SensorPacket
from cv_bridge import CvBridge, CvBridgeError

DIR = os.path.dirname(__file__)
if DIR == "":
    DIR = '.'

class ApproximateTimeSynchronizer(TimeSynchronizer):

    def __init__(self, fs, queue_size, slop):
        TimeSynchronizer.__init__(self, fs, queue_size)
        self.slop = rospy.Duration.from_sec(slop)

    def add(self, msg, my_queue):
        self.lock.acquire()
        my_queue[msg.header.stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        for vv in itertools.product(*[list(q.keys()) for q in self.queues]):
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
        self.lock.release()

class CameraTabPanel(wx.Panel):
    def __init__(self, parent):
        self.parent = parent
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour("white")
        
        self.lock = threading.Lock()

        self.dataL = np.zeros((240,320,3))
        self.dataR = np.zeros((240,320,3))
        
        self.initPlot()
        
        self.hbox = wx.BoxSizer(wx.HORIZONTAL)
        self.hbox.Add(self.canvasL, proportion=1, border=2, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.hbox.Add(self.canvasR, proportion=1, border=2, flag=wx.RIGHT | wx.TOP | wx.GROW)
        self.SetSizer(self.hbox)
        self.hbox.Fit(self)

        self.inputL = rospy.get_param('~camera_input_left', '/video/left/rgb')
        self.inputR = rospy.get_param('~camera_input_right', '/video/right/rgb')
        
        self.bridge = CvBridge()

        id = wx.NewId()
        wx.RegisterId(id)
        self.refreshTimer = wx.Timer(self, id)
        self.Bind(wx.EVT_TIMER, self.onRefresh, self.refreshTimer, id)
        self.refreshTimer.Start(milliseconds=100, oneShot=False)

        subscriberLeft = message_filters.Subscriber(self.inputL, Image)
        subscriberRight = message_filters.Subscriber(self.inputR, Image)
        
        sync = ApproximateTimeSynchronizer((subscriberLeft, subscriberRight), queue_size=4, slop=0.5)
        sync.registerCallback(self.callback)
        
    def callback(self, msgLeft, msgRight):
        self.callbackLeft(msgLeft)
        self.callbackRight(msgRight)

    def callbackLeft(self, msg):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            (rows,cols,channels) = img.shape
            
            with self.lock:
                self.dataL = img
        except Exception, e:
          rospy.logerr('Error occurred while converting image: %s' % (e.message))

    def callbackRight(self, msg):
        try:
              img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
              (rows,cols,channels) = img.shape
              with self.lock:
                  self.dataR = img
        except Exception, e:
          rospy.logerr('Error occurred while converting image: %s' % (e.message))

    def initPlot(self):
        dpi = 100
        facecolor='white'
        size = (3.0, 3.0)
        
        self.figL = Figure(size, frameon=True, facecolor=facecolor, dpi=dpi)
        self.axL = self.figL.add_subplot(111)
        self.axL.set_title('Left Camera', size=12)
        self.axL.axis('off')
        self.imL = self.axL.imshow(self.dataL)
        self.canvasL = FigCanvas(self, -1, self.figL)
        
        self.figR = Figure(size, frameon=True, facecolor=facecolor, dpi=dpi)
        self.axR = self.figR.add_subplot(111)
        self.axR.set_title('Right Camera', size=12)
        self.axR.axis('off')
        self.imR = self.axR.imshow(self.dataR)
        self.canvasR = FigCanvas(self, -1, self.figR)
        
        self.drawPlot()

    def onRefresh(self, event):
        
#         # GENERATE RANDOM DATA
#         self.dataL = random.random((240,320,3))
#         self.dataR = random.random((240,320,3))
        
        if self.parent.GetPageText(self.parent.GetSelection()) == 'Camera':
            self.drawPlot()

    def drawPlot(self):
        
        with self.lock:
            self.imL.set_array(self.dataL)
            self.imR.set_array(self.dataR)
            
        self.canvasL.draw()
        self.canvasR.draw()

class AudioTabPanel(wx.Panel):
    
    def __init__(self, parent, windowSize=100000):
        self.parent = parent
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour("white")
        
        self.lock = threading.Lock()

        self.initDone = False
        self.fs = 16000.0
        self.windowSize = windowSize
        self.dataL = np.zeros(windowSize)
        self.sizeL = 1
        self.dataR = np.zeros(windowSize)
        self.sizeR = 1
        
        self.initPlot()
        
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvasL, proportion=1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.vbox.Add(self.canvasR, proportion=1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.vbox)
        self.vbox.Fit(self)

        self.inputL = rospy.get_param('~audio_input_left', '/audio/left/raw')
        self.inputR = rospy.get_param('~audio_input_right', '/audio/right/raw')
        
        id = wx.NewId()
        wx.RegisterId(id)
        self.refreshTimer = wx.Timer(self, id)
        self.Bind(wx.EVT_TIMER, self.onRefresh, self.refreshTimer, id)        
        self.refreshTimer.Start(milliseconds=500, oneShot=False)

        subscriberLeft = message_filters.Subscriber(self.inputL, AudioPacket)
        subscriberRight = message_filters.Subscriber(self.inputR, AudioPacket)
        
        sync = ApproximateTimeSynchronizer((subscriberLeft, subscriberRight), queue_size=4, slop=0.25)
        sync.registerCallback(self.callback)
        
    def callback(self, msgLeft, msgRight):
        self.callbackLeft(msgLeft)
        self.callbackRight(msgRight)

    def callbackLeft(self, msg):
        
        # Convert to numpy array and normalize in range [-1,1]
        audio = np.array(msg.data)
        audio = audio / 32767.0
        
        with self.lock:
            if not self.initDone:
                self.fs = float(msg.fs)
                self.initDone = True
            
            newDataSize = len(audio)
            if self.sizeL + newDataSize <= self.windowSize:
                self.dataL[self.sizeL:self.sizeL+newDataSize] = audio
            else:
                shift = min(newDataSize, self.sizeL + newDataSize - self.windowSize)
                self.dataL = np.roll(self.dataL, -shift)
                self.dataL[self.windowSize-newDataSize:] = audio
            self.sizeL += newDataSize

    def callbackRight(self, msg):
        # Convert to numpy array and normalize in range [-1,1]
        audio = np.array(msg.data)
        audio = audio / 32767.0

        with self.lock:
            if not self.initDone:
                self.fs = float(msg.fs)
                self.initDone = True
            
            newDataSize = len(audio)
            if self.sizeR + newDataSize <= self.windowSize:
                self.dataR[self.sizeR:self.sizeR+newDataSize] = audio
            else:
                shift = min(newDataSize, self.sizeR + newDataSize - self.windowSize)
                self.dataR = np.roll(self.dataR, -shift)
                self.dataR[self.windowSize-newDataSize:] = audio
            self.sizeR += newDataSize

    def initPlot(self):
        dpi = 100
        facecolor='white'
        size = (3.0, 2.0)
        
        self.figL = Figure(size, frameon=True, facecolor=facecolor, dpi=dpi)
        self.axL = self.figL.add_subplot(111)
        self.axL.set_title('Left Microphone', size=12)
        self.axL.set_xlabel("Time [s]", fontsize=10)
        self.axL.set_ylabel("Amplitude", fontsize=10)
        self.axL.set_axis_bgcolor(facecolor)
        pylab.setp(self.axL.get_xticklabels(), fontsize=8)
        pylab.setp(self.axL.get_yticklabels(), fontsize=8)
        self.plotL = self.axL.plot(self.dataL, linewidth=1, color='r')[0]
        self.figL.subplots_adjust(bottom=0.18)
        self.canvasL = FigCanvas(self, -1, self.figL)

        self.figR = Figure(size, frameon=True, facecolor=facecolor, dpi=dpi)
        self.axR = self.figR.add_subplot(111)
        self.axR.set_title('Right Microphone', size=12)
        self.axR.set_xlabel("Time [s]", fontsize=10)
        self.axR.set_ylabel("Amplitude", fontsize=10)
        self.axR.set_axis_bgcolor(facecolor)
        pylab.setp(self.axR.get_xticklabels(), fontsize=8)
        pylab.setp(self.axR.get_yticklabels(), fontsize=8)
        self.plotR = self.axR.plot(self.dataR, linewidth=1, color='b')[0]
        self.figR.subplots_adjust(bottom=0.18)
        self.canvasR = FigCanvas(self, -1, self.figR)
        
        self.canvasL.draw()
        self.canvasR.draw()
        self.drawPlot()

    def onRefresh(self, event):

#         # GENERATE RANDOM DATA
#         newDataSize = 1024
#         t = np.arange(self.sizeR, self.sizeR + newDataSize)
#         omega1 = 0.001
#         omega2 = 0.0001
#         audio = np.sin(2*np.pi*omega1*t) * np.sin(2*np.pi*omega2*t) * 32767.0
#         msg = type('msg', (object,), dict(data=audio))
#         self.callbackLeft(msg)
#         self.callbackRight(msg)

        if self.parent.GetPageText(self.parent.GetSelection()) == 'Audio':
            self.drawPlot()

    def drawPlot(self):
        
        with self.lock:
            if self.sizeL <= self.windowSize:
                xmin = 0
                xmax = self.sizeL
            else:
                xmin = self.sizeL - self.windowSize
                xmax = xmin + min(self.sizeL,self.windowSize)
            self.axL.set_xbound(lower=xmin/self.fs, upper=xmax/self.fs)
            self.axL.set_ybound(lower=-1.1, upper=1.1)
            self.plotL.set_xdata((xmin + np.arange(xmax - xmin))/self.fs)
            self.plotL.set_ydata(self.dataL[:min(self.sizeL,self.windowSize)])
         
            if self.sizeR <= self.windowSize:
                xmin = 0
                xmax = self.sizeR
            else:
                xmin = self.sizeR - self.windowSize
                xmax = xmin + min(self.sizeR,self.windowSize)
            self.axR.set_xbound(lower=xmin/self.fs, upper=xmax/self.fs)
            self.axR.set_ybound(lower=-1.1, upper=1.1)
            self.plotR.set_xdata((xmin + np.arange(xmax - xmin))/self.fs)
            self.plotR.set_ydata(self.dataR[:min(self.sizeR,self.windowSize)])
                
        self.figL.canvas.draw()
        self.figL.canvas.flush_events()
        
        self.figR.canvas.draw()
        self.figR.canvas.flush_events()

class ImuTabPanel(wx.Panel):
    def __init__(self, parent, windowSize=512):
        self.parent = parent
        wx.Panel.__init__(self, parent=parent)
        self.SetBackgroundColour("white")
        
        self.initDone = False
        self.fs = 20.0
        self.lock = threading.Lock()

        self.windowSize = windowSize
        self.data = np.zeros((windowSize, 9))
        self.size = 1
        
        self.initPlot()
        
        self.vbox = wx.BoxSizer(wx.VERTICAL)
        self.vbox.Add(self.canvas, proportion=1, flag=wx.LEFT | wx.TOP | wx.GROW)
        self.SetSizer(self.vbox)
        self.vbox.Fit(self)

        self.input = rospy.get_param('~imu_input', '/imu/raw')
        
        rospy.Subscriber(self.input, ImuPacket, self.callback)

        id = wx.NewId()
        wx.RegisterId(id)
        self.refreshTimer = wx.Timer(self, id)
        self.Bind(wx.EVT_TIMER, self.onRefresh, self.refreshTimer, id)        
        self.refreshTimer.Start(milliseconds=500, oneShot=False)

    def callback(self, msg):
        data = np.array([[msg.accelX, msg.accelY, msg.accelZ, msg.gyroX, msg.gyroY, msg.gyroZ, msg.magX, msg.magY, msg.magZ]]);

        with self.lock:
            if not self.initDone:
                self.fs = float(msg.fs)
                self.initDone = True
            
            newDataSize = 1
            if self.size + newDataSize <= self.windowSize:
                self.data[self.size:self.size+newDataSize,:] = data
            else:
                shift = min(newDataSize, self.size + newDataSize - self.windowSize)
                self.data = np.roll(self.data, -shift, axis=0)
                self.data[self.windowSize-newDataSize:,:] = data
            self.size += newDataSize

    def initPlot(self):
        dpi = 100
        facecolor='white'
        size = (3.0, 5.0)
        
        self.fig = Figure(size, frameon=True, facecolor=facecolor, dpi=dpi)
        
        self.axA = self.fig.add_subplot(311)
        self.axA.set_title("Accelerometer", size=12)
        self.axA.set_xlabel("Time [s]", fontsize=10)
        self.axA.set_ylabel("Amplitude", fontsize=10)
        self.axA.set_axis_bgcolor(facecolor)
        pylab.setp(self.axA.get_xticklabels(), fontsize=8)
        pylab.setp(self.axA.get_yticklabels(), fontsize=8)
        
        self.axG = self.fig.add_subplot(312)
        self.axG.set_title("Gyroscope", size=12)
        self.axG.set_xlabel("Time [s]", fontsize=10)
        self.axG.set_ylabel("Amplitude", fontsize=10)
        self.axG.set_axis_bgcolor(facecolor)
        pylab.setp(self.axG.get_xticklabels(), fontsize=8)
        pylab.setp(self.axG.get_yticklabels(), fontsize=8)
        
        self.axM = self.fig.add_subplot(313)
        self.axM.set_title("Magnetometer", size=12)
        self.axM.set_xlabel("Time [s]", fontsize=10)
        self.axM.set_ylabel("Amplitude", fontsize=10)
        
        self.axM.set_axis_bgcolor(facecolor)
        pylab.setp(self.axM.get_xticklabels(), fontsize=8)
        pylab.setp(self.axM.get_yticklabels(), fontsize=8)
        
        self.plotA = self.axA.plot(self.data[:,0],'-r', self.data[:,1],'-g', self.data[:,2],'-b')
        self.plotG = self.axG.plot(self.data[:,3],'-r', self.data[:,4],'-g', self.data[:,5],'-b')
        self.plotM = self.axM.plot(self.data[:,6],'-r', self.data[:,7],'-g', self.data[:,8],'-b')
    
        self.axA.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left', prop={'size':8})
        self.axG.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left', prop={'size':8})
        self.axM.legend(['x-axis', 'y-axis', 'z-axis'], loc='upper left', prop={'size':8})
    
        self.fig.subplots_adjust(hspace = 0.75)
        self.canvas = FigCanvas(self, -1, self.fig)
        
        self.canvas.draw()
        self.drawPlot()

    def onRefresh(self, event):

#         # GENERATE RANDOM DATA
#         newDataSize = 8
#         if self.size + newDataSize <= self.windowSize:
#             self.data[self.size:self.size+newDataSize,:] = random.random((newDataSize,9))
#         else:
#             shift = min(newDataSize, self.size + newDataSize - self.windowSize)
#             self.data = np.roll(self.data, -shift, axis=0)
#             self.data[self.windowSize-newDataSize:,:] = random.random((newDataSize,9))
#         self.size += newDataSize

        if self.parent.GetPageText(self.parent.GetSelection()) == 'Inertial Measurement Unit':
            self.drawPlot()

    def drawPlot(self):
        
        with self.lock:
            if self.size < self.windowSize:
                xmin = 0
                xmax = self.size
            else:
                xmin = self.size - self.windowSize
                xmax = xmin + min(self.size,self.windowSize)
            self.axA.set_xbound(lower=xmin/self.fs, upper=xmax/self.fs)
            self.axA.set_ybound(lower=-2.1, upper=2.1)
            self.axG.set_xbound(lower=xmin/self.fs, upper=xmax/self.fs)
            self.axG.set_ybound(lower=-100.1, upper=100.1)
            self.axM.set_xbound(lower=xmin/self.fs, upper=xmax/self.fs)
            self.axM.set_ybound(lower=-2.1, upper=2.1)
            
            xdata = (xmin + np.arange(xmax - xmin))/self.fs
            
            for i in range(0, 3):
                self.plotA[i].set_xdata(xdata)
                self.plotA[i].set_ydata(self.data[0:min(self.size,self.windowSize), i])
            
            for i in range(0, 3):
                self.plotG[i].set_xdata(xdata)
                self.plotG[i].set_ydata(self.data[0:min(self.size,self.windowSize), i + 3])
            
            for i in range(0, 3):
                self.plotM[i].set_xdata(xdata)
                self.plotM[i].set_ydata(self.data[0:min(self.size,self.windowSize), i + 6])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

class IRobotCreateTabPanel(wx.Panel):
    
    CHARGE_STATES = {   0 : "Not charging",
                    1 : "Reconditioning Charging",
                    2 : "Full Charging",
                    3 : "Trickle Charging",
                    4 : "Waiting",
                    5 : "Charging Fault Condition"  }
    
    def __init__(self, parent):
        self.parent = parent
        wx.Panel.__init__(self, parent=parent)

        self.lock = threading.Lock()
        
        self.state = None
        
        self.SetBackgroundColour("white")
        self.initRobotPanel()
        
        font = wx.Font(14, wx.DECORATIVE, wx.NORMAL, wx.NORMAL)
        
        self.dockStatus = wx.StaticText(self, label="Status: Not Available")
        self.dockStatus.SetFont(font)
        self.batteryStatus = wx.StaticText(self, label="Battery charge: Not Available")
        self.batteryStatus.SetFont(font)
        self.powerStatus = wx.StaticText(self, label="Power: Not Available")
        self.powerStatus.SetFont(font)
        self.chargingStatus = wx.StaticText(self, label="Charging state: Not Available")
        self.chargingStatus.SetFont(font)
        self.internalChargerStatus = wx.StaticText(self, label="Internal charger: Not Available")
        self.internalChargerStatus.SetFont(font)
        
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(self.dockStatus, 0, border=15, flag=wx.LEFT | wx.TOP)
        self.sizer.Add(self.batteryStatus, 0, border=15, flag=wx.LEFT | wx.TOP)
        self.sizer.Add(self.powerStatus, 0, border=15, flag=wx.LEFT | wx.TOP)
        self.sizer.Add(self.chargingStatus, 0, border=15, flag=wx.LEFT | wx.TOP)
        self.sizer.Add(self.internalChargerStatus, 0, border=15, flag=wx.LEFT | wx.TOP)
        self.sizer.AddStretchSpacer(1)
        self.sizer.Add(self.robotPanel, 0, border=5, flag=wx.ALIGN_CENTER)
        self.sizer.AddStretchSpacer(1)
        self.SetSizer(self.sizer)
        self.sizer.Fit(self)

        self.input = rospy.get_param('~create_input', '/irobot_create/sensorPacket')
        
        rospy.Subscriber(self.input, SensorPacket, self.callback)

        id = wx.NewId()
        wx.RegisterId(id)
        self.refreshTimer = wx.Timer(self, id)
        self.Bind(wx.EVT_TIMER, self.onRefresh, self.refreshTimer, id)        
        self.refreshTimer.Start(milliseconds=500, oneShot=False)

    def initRobotPanel(self):
        self.robotBitmap = wx.Bitmap(DIR + '/images/irobot.png', wx.BITMAP_TYPE_PNG)
       
        self.robotPanel = wx.Panel(self, wx.ID_ANY, size=self.robotBitmap.GetSize())
        self.robotPanel.SetBackgroundColour("white")
        
        self.bumperLeftBitmap = wx.Bitmap(DIR + '/images/bumper_left.png', wx.BITMAP_TYPE_PNG)
        self.bumperRightBitmap = wx.Bitmap(DIR + '/images/bumper_right.png', wx.BITMAP_TYPE_PNG)
        self.virtualWallBitmap = wx.Bitmap(DIR + '/images/virtual_wall.png', wx.BITMAP_TYPE_PNG)
        self.wallBitmap = wx.Bitmap(DIR + '/images/wall.png', wx.BITMAP_TYPE_PNG)
        
    def callback(self, msg):
        with self.lock:
            self.state = msg

    def update(self):
        
        dc = wx.BufferedPaintDC(self.robotPanel)
        dc.Clear()

        with self.lock:
            if self.state != None:
                dc.DrawBitmap(self.robotBitmap, 0, 0)
                
                if self.state.bumpLeft:
                    dc.DrawBitmap(self.bumperLeftBitmap, 0, 0)
                if self.state.bumpRight:
                    dc.DrawBitmap(self.bumperRightBitmap, 0, 0)
                if self.state.virtualWall:
                    dc.DrawBitmap(self.virtualWallBitmap, 0, 0)
                if self.state.wall:
                    dc.DrawBitmap(self.wallBitmap, 0, 0)
                    
                if self.state.homeBase:
                    self.dockStatus.SetLabel("Status: Docked")
                else: 
                    self.dockStatus.SetLabel("Status: Free")
                    
                if self.state.internalCharger:
                    self.internalChargerStatus.SetLabel("Internal charger: Connected")
                else:
                    self.internalChargerStatus.SetLabel("Internal charger: Disconnected")
                    
                self.chargingStatus.SetLabel("Charging state: %s" % (self.CHARGE_STATES[self.state.chargingState]))
                
                batteryCharge = float(self.state.batteryCharge) / self.state.batteryCapacity * 100
                self.batteryStatus.SetLabel("Battery charge: %3.0f%%" % (batteryCharge))
                self.powerStatus.SetLabel("Power: %2.1fV at %4.0fmA" % (float(self.state.voltage) / 1000, abs(float(self.state.current))))
            else:
                dc.DrawBitmap(self.robotBitmap, 0, 0)
        
        self.sizer.Layout()
        
        del dc

    def onRefresh(self, event):
        
#         # GENERATE RANDOM DATA
#         msg = type('msg', (object,), dict(bumpLeft=False, bumpRight=False, virtualWall=False, wall=False, homeBase=False, internalCharger=False, batteryCharge=80, batteryCapacity=100, voltage=134, current=500, chargingState=0))
#         msg.bumpLeft = random.random() > 0.5
#         msg.bumpRight = random.random() > 0.5    
#         msg.virtualWall = random.random() > 0.5
#         msg.wall = random.random() > 0.5
#         msg.homeBase = random.random() > 0.5
#         msg.internalCharger = random.random() > 0.5
#         msg.batteryCharge=80 * random.random()
#         msg.batteryCapacity=100
#         msg.voltage=134 * random.random()
#         msg.current=500 * random.random()
#         msg.chargingState = random.randint(0, 5)
#         self.callback(msg)
        
        if self.parent.GetPageText(self.parent.GetSelection()) == 'Onboard Sensors':
            self.update()
    
class VisualizationFrame(wx.Frame):

    title = 'IRobot Create: visualization application'
    
    def __init__(self):
        wx.Frame.__init__(self, None, wx.ID_ANY, self.title, size=(800,600))
        
        self.createMenu()
        self.createStatusBar()
        self.createMainPanel()
        
        self.SetIcon(wx.Icon(DIR + '/icons/application.ico', wx.BITMAP_TYPE_ICO))
        
    def createMenu(self):
        self.menuBar = wx.MenuBar()
        
        menuFile = wx.Menu()
        
        #mConnect = wx.MenuItem(menuFile, 101, '&Connect\tCtrl+C', 'Connect to Robot')
        #mConnect.SetBitmap(wx.Image('icons/connect.ico', wx.BITMAP_TYPE_ICO).ConvertToBitmap())
        #menuFile.AppendItem(mConnect)
        #self.Bind(wx.EVT_MENU, self.onConnect, mConnect)
        #menuFile.AppendSeparator()
        
        mExit = wx.MenuItem(menuFile, 103, '&Quit\tCtrl+Q', 'Quit the Application')
        mExit.SetBitmap(wx.Image(DIR + '/icons/stock_exit.png', wx.BITMAP_TYPE_PNG).ConvertToBitmap())
        menuFile.AppendItem(mExit)
        
        self.Bind(wx.EVT_MENU, self.onExit, mExit)
        
        self.menuBar.Append(menuFile, "&File")
        self.SetMenuBar(self.menuBar)

    def createMainPanel(self):
        self.panel = wx.Panel(self)

        self.notebook = wx.Notebook(self.panel)
        
        self.cameraTab = CameraTabPanel(self.notebook)
        self.notebook.AddPage(self.cameraTab, "Camera")
 
        self.audioTab = AudioTabPanel(self.notebook)
        self.notebook.AddPage(self.audioTab, "Audio")
 
        self.imuTab = ImuTabPanel(self.notebook)
        self.notebook.AddPage(self.imuTab, "Inertial Measurement Unit")
 
        self.robotTab = IRobotCreateTabPanel(self.notebook)
        self.notebook.AddPage(self.robotTab, "Onboard Sensors")
 
        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(self.notebook, 1, wx.ALL|wx.EXPAND, 5)
        self.panel.SetSizer(sizer)
        self.Layout()
        
        self.notebook.Bind(wx.EVT_NOTEBOOK_PAGE_CHANGED, self.onTabChange)
        
        self.Show()
        
    def createStatusBar(self):
        self.statusBar = self.CreateStatusBar()
        self.statusBar.SetFieldsCount(1)
        
        rosMasterUri = os.environ['ROS_MASTER_URI']
        self.statusBar.SetStatusText('Connected to %s' % (rosMasterUri), 0)
    
    def onExit(self, event):
        self.Destroy()
        
    def onTabChange(self, event):
        #page = self.GetPageText(evt.GetSelection())
        event.Skip()
    
    def flashStatusMessage(self, msg, flashLenMs=1500):
        self.statusBar.SetStatusText(msg)
        self.timeroff = wx.Timer(self)
        self.Bind(
            wx.EVT_TIMER, 
            self.onFlashStatusOff, 
            self.timeroff)
        self.timeroff.Start(flashLenMs, oneShot=True)
    
    def onFlashStatusOff(self, event):
        self.statusBar.SetStatusText('')

if __name__ == '__main__':
    
    rospy.init_node('viewer')
    
    try:
        app = wx.App()
        app.frame = VisualizationFrame()
        app.frame.Show()
        app.MainLoop()

    except rospy.ROSInterruptException: pass
