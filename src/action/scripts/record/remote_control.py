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


import os, struct, array
import sys
import random
import numpy as np
import time
import subprocess
from fcntl import ioctl

import rospy
import rospkg

from action.behaviours import BehaviourController, Behaviour, MotorAction
from create.srv import Beep, Leds, Brake


class Joystick:
    
    axis_names = {
        0x00 : 'x',
        0x01 : 'y',
        0x02 : 'z',
        0x03 : 'rx',
        0x04 : 'ry',
        0x05 : 'rz',
        0x06 : 'trottle',
        0x07 : 'rudder',
        0x08 : 'wheel',
        0x09 : 'gas',
        0x0a : 'brake',
        0x10 : 'hat0x',
        0x11 : 'hat0y',
        0x12 : 'hat1x',
        0x13 : 'hat1y',
        0x14 : 'hat2x',
        0x15 : 'hat2y',
        0x16 : 'hat3x',
        0x17 : 'hat3y',
        0x18 : 'pressure',
        0x19 : 'distance',
        0x1a : 'tilt_x',
        0x1b : 'tilt_y',
        0x1c : 'tool_width',
        0x20 : 'volume',
        0x28 : 'misc',
    }
    
    button_names = {
        0x120 : 'trigger',
        0x121 : 'thumb',
        0x122 : 'thumb2',
        0x123 : 'top',
        0x124 : 'top2',
        0x125 : 'pinkie',
        0x126 : 'base',
        0x127 : 'base2',
        0x128 : 'base3',
        0x129 : 'base4',
        0x12a : 'base5',
        0x12b : 'base6',
        0x12f : 'dead',
        0x130 : 'a',
        0x131 : 'b',
        0x132 : 'c',
        0x133 : 'x',
        0x134 : 'y',
        0x135 : 'z',
        0x136 : 'tl',
        0x137 : 'tr',
        0x138 : 'tl2',
        0x139 : 'tr2',
        0x13a : 'select',
        0x13b : 'start',
        0x13c : 'mode',
        0x13d : 'thumbl',
        0x13e : 'thumbr',
    
        0x220 : 'dpad_up',
        0x221 : 'dpad_down',
        0x222 : 'dpad_left',
        0x223 : 'dpad_right',
    
        # XBox 360 controller uses these codes.
        0x2c0 : 'dpad_left',
        0x2c1 : 'dpad_right',
        0x2c2 : 'dpad_up',
        0x2c3 : 'dpad_down',
    }
    
    def __init__(self, name='/dev/input/js0'):
        self.name = name
        self.axis_map = []
        self.button_map = []
        self.axis_states = {}
        self.button_states = {}
        
        # Open the joystick device
        self.jsdev = os.open(self.name, os.O_RDONLY | os.O_NONBLOCK)
        
        # Get the device name.
        buf = array.array('c', ['\0'] * 64)
        ioctl(self.jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
        js_name = buf.tostring()
        rospy.loginfo('Device name: %s' % js_name)
        
        # Get number of axes and buttons.
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a11, buf) # JSIOCGAXES
        num_axes = buf[0]
        
        buf = array.array('B', [0])
        ioctl(self.jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
        num_buttons = buf[0]
        
        # Get the axis map.
        buf = array.array('B', [0] * 0x40)
        ioctl(self.jsdev, 0x80406a32, buf) # JSIOCGAXMAP
        
        for axis in buf[:num_axes]:
            axis_name = self.axis_names.get(axis, 'unknown(0x%02x)' % axis)
            self.axis_map.append(axis_name)
            self.axis_states[axis_name] = 0.0
        
        # Get the button map.
        buf = array.array('H', [0] * 200)
        ioctl(self.jsdev, 0x80406a34, buf) # JSIOCGBTNMAP
        
        for btn in buf[:num_buttons]:
            btn_name = self.button_names.get(btn, 'unknown(0x%03x)' % btn)
            self.button_map.append(btn_name)
            self.button_states[btn_name] = 0
        
        rospy.loginfo('%d axes found: %s' % (num_axes, ', '.join(self.axis_map)))
        rospy.loginfo('%d buttons found: %s' % (num_buttons, ', '.join(self.button_map)))
        

    def update(self):
        
        evbuf = None
        try:
            while True:
                evbuf = os.read(self.jsdev,8)
                time, value, type, number = struct.unpack('IhBB', evbuf)
        
                if type & 0x80:
                     rospy.logdebug("(initial)")
        
                if type & 0x01:
                    button = self.button_map[number]
                    if button:
                        self.button_states[button] = value
                        if value:
                            rospy.logdebug("%s pressed" % (button))
                        else:
                            rospy.logdebug("%s released" % (button))
        
                if type & 0x02:
                    axis = self.axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        self.axis_states[axis] = fvalue
                        rospy.logdebug("%s: %.3f" % (axis, fvalue))
    
            print button_states
    
        except OSError:
            pass

    def getState(self):
        return (self.axis_states['x'], -self.axis_states['y'])

class RemoteControl(Behaviour):
    
    SPEED = 200 # mm/s
    
    def __init__(self, dev, priority=0, controller=None):
        self.joystick = Joystick(dev)
        self.enabled = True
        self.mode = None
        self.recording = False
        #rospy.wait_for_service('/irobot_create/leds')
        #rospy.wait_for_service('irobot_create/beep')
        self.beepControl = rospy.ServiceProxy('/irobot_create/beep', Beep)
        Behaviour.__init__(self, priority, controller)
        self.rospack = rospkg.RosPack()
        self.durationOfRecording = rospy.Duration.from_sec(900.1)
        self.timeStartRecord = rospy.Time.now()
        self.recordingNumber = 0
        self.rosRecordingProcess = None 
        self.rosbagSaveLoc= rospy.get_param('~rosbag_loc', '/root/work/rosbags')
        self.oldXInput = 0
        self.oldYInput = 0

    def executeControl(self, state):
        
        # Update joystick states
        #lastEnableActivated = self.joystick.button_states['base4'] == 1
        lastEnableActivated = True 
        self.joystick.update()
        #enableActivated = self.joystick.button_states['base4'] == 1
        enableActivated = True
        
        if self.enabled and enableActivated and not lastEnableActivated:
            # If joystick is not activated
            self.enabled = False
            rospy.loginfo("Joystick deactivated")
        elif not self.enabled and enableActivated and not lastEnableActivated:
            # If joystick is reactivated
            self.enabled = True
            rospy.loginfo("Joystick reactivated")
        
        if not self.enabled:
            return None
        
        
        if 'b' in self.joystick.button_states and 'y' in self.joystick.button_states:
            recordActivation = self.joystick.button_states['b'] and self.joystick.button_states['y']
        elif 'base' in self.joystick.button_states:
            recordActivation = self.joystick.button_states['base']
            
        if recordActivation == 1 and self.recording == False:
            rospy.logwarn("Will start Recording!")
            self.timeStartRecord = rospy.Time.now()
            self.recording = True
            args="--regex '/(diagnostics|rosout|rosout_agg|imu/(.*)|irobot_create/(.*)|audio/(.*)|video/(.*))' --quiet --buffsize=128 "
            startLnhRecording = 'rosbag record ' + args + ' --duration=15m -O' + self.rosbagSaveLoc + '/session_' + time.strftime("%Y%m%d_%H%M%S", time.localtime()) + '.bag'

            # Launch command in a subprocess
            self.rosRecordingProcess = subprocess.Popen(startLnhRecording, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            self.recordingNumber += 1
            #Blink led once
            try :
                #respBag = self.recordBag(10)
                self.beepControl(1)
            except rospy.ServiceException, e:
                rospy.logwarn( "Service call failed: %s", e)
           

        #interrupt recording
        
        if 'x' in self.joystick.button_states and 'a' in self.joystick.button_states:
            recordCancelation = self.joystick.button_states['x'] and self.joystick.button_states['a']
        elif 'base2' in self.joystick.button_states:
            recordCancelation = self.joystick.button_states['base2']
        
        if recordCancelation == 1 and self.recording == True:
            #Must SIGINT child process ourselves, since rosbag record creates a few
            ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % self.rosRecordingProcess.pid, shell=True, stdout=subprocess.PIPE)
            ps_output = ps_command.stdout.read()
            retcode = ps_command.wait()
            assert retcode == 0, "ps command returned %d" % retcode
            
            ps_command = subprocess.Popen("ps -o pid --ppid %d --noheaders" % int(ps_output), shell=True, stdout=subprocess.PIPE)
            ps_output = ps_command.stdout.read()
            retcode = ps_command.wait()
            assert retcode == 0, "ps command returned %d" % retcode
            
            for pid_str in ps_output.split("\n")[:-1]:
                os.kill(int(pid_str), subprocess.signal.SIGINT)
            #self.rosRecordingProcess.send_signal(subprocess.signal.SIGINT)
            self.rosRecordingProcess.terminate()
            self.recording = False
            rospy.logwarn("Stopped recording rosbag")        
            #Blink led twice
            try :
                self.beepControl(2)
            except rospy.ServiceException, e:
                rospy.logwarn( "Service call failed: %s", e)
        
        #stop lock on recording after 15min
        if (rospy.Time.now() - self.timeStartRecord ) > self.durationOfRecording:
            if self.recording == True:
                self.recording = False
                self.beepControl(3)

        x,y = self.joystick.getState()   
        left = 0.0
        right = 0.0

        #adding and change checker minimum movement requirement
        if (x != self.oldXInput or y != self.oldYInput) and (abs(x) > 0.2 or abs(y) > 0.2):
           
            # Avoid small controller deviations, that make it hard to go in a straight line
            if abs(x) < 0.2:
                x = 0.0
            if abs(y) < 0.2:
                y = 0.0

            # Conversion algorithm adapted from:
            # http://www.goodrobot.com/en/2009/09/tank-drive-via-joystick-control/
            angle = np.arccos(np.abs(x)/np.sqrt(x**2 + y**2)) * 180.0 / np.pi
            tcoeff = -1 + (angle/90.0) * 2
            turn = tcoeff * np.abs(np.abs(y) - np.abs(x))
            turn = np.round(turn*100.0)/100.0
            move = np.max((np.abs(y),np.abs(x)))

            if (x >= 0.0 and y >= 0.0) or (x < 0.0 and y < 0.0):
                left = move
                right = turn
            else:
                right = move
                left = turn
        
            if (y < 0.0):
                left = 0.0 - left
                right = 0.0 - right
                
        leftValue = left * self.SPEED
        rightValue = right * self.SPEED
        return MotorAction(leftValue, rightValue)
    
    def execute(self, state):
        
        # Activation of behaviour
        if not self.isActive and not self.controller.isActiveBehaviour(ignored=[self]):
            self.activate()
            self.mode = 'control'
            rospy.logdebug("Remote control activated")
        elif self.isActive and self.controller.isActiveBehaviour(ignored=[self]):
            self.deactivate()
            self.mode = None
            rospy.logdebug("Remote control deactivated")
            
        # Execution branching depending on current mode
        action = None
        if self.mode == 'control':
            action = self.executeControl(state)
        return action

if __name__ == '__main__':
    
    try:
        rospy.init_node('remote_control', log_level=rospy.INFO)
        dev = rospy.get_param('~joystick_dev', '/dev/input/js0')
        rate = rospy.get_param('~rate', 20.0)
        controller = BehaviourController(rate, stateless=True)
        controller.addBehaviour(RemoteControl(dev), priority=0)
        controller.spin()

    except rospy.ROSInterruptException: pass
