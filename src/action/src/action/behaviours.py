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
import random
import math
import numpy as np
import threading
import time
from time import sleep

import rospy
from irobot_create.msg import Contact, MotorSpeed
from sensor_msgs.msg import BatteryState
from irobot_create.srv import Tank, Dock
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class MotorAction:
    def __init__(self, leftSpeed, rightSpeed):
        assert np.abs(leftSpeed) <= 500.0 and np.abs(rightSpeed) <= 500.0
        self.leftSpeed = int(leftSpeed)
        self.rightSpeed = int(rightSpeed)

class DockAction:
    pass

class RobotState:
    
    def __init__(self, contact, battery, odom):
        self.contact = contact
        self.battery = battery
        self.odom = odom
        
        self.lastPosition = None
        self.distance = 0.0

    def getOdomAngle(self):
        # Quaternion to X-Y plane angle:
        #         quaternion.x = 0.0 
        #         quaternion.y = 0.0
        #         quaternion.z = sin(self.th/2)
        #         quaternion.w = cos(self.th/2)
        th = 2.0 * np.arctan2(self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
        return th
        
    def _updateDistance(self):
        if self.lastPosition == None:
            self.lastPosition = self.odom.pose.pose.position
        
        newPosition = self.odom.pose.pose.position
        distance = np.sqrt((self.lastPosition.x - newPosition.x)**2 + 
                           (self.lastPosition.y - newPosition.y)**2 + 
                           (self.lastPosition.z - newPosition.z)**2)
        self.distance += distance
        
    def getOdomDistance(self):
        #return in mm
        return self.distance*1000.0
    
class Behaviour:

    def __init__(self, priority=0, controller=None):
        self.controller = controller
        self.priority = priority
        self.isActive = False
        self.lastActivated = None
        self.lastDeactivated = None
        
    def setPriority(self, priority):
        self.priority = priority
        
    def activate(self):
        self.isActive = True
        self.lastActivated = time.time()
        
    def deactivate(self):
        self.isActive = False
        self.lastDeactivated = time.time()
        
class BehaviourController:
    
    def __init__(self, rate=None):

        self.rate = rate
        self.lock = threading.Lock()
        self.isDocking = False
        self.lastMotorAction = None
        self.state = RobotState(Contact(),BatteryState(),Odometry())
        
        rospy.wait_for_service('/irobot_create/tank')
        self.tank = rospy.ServiceProxy('/irobot_create/tank', Tank)
        self.dock = rospy.ServiceProxy('/irobot_create/dock', Dock)
        
        self.input = rospy.get_param('~input', '/irobot_create')
        rospy.Subscriber(self.input + '/battery', BatteryState, BehaviourController.callback, self)
        rospy.Subscriber(self.input + '/contact', Contact, BehaviourController.callback, self)
        rospy.Subscriber(self.input + '/odom', Odometry, BehaviourController.callback, self)
        self.rawPub = rospy.Publisher('/irobot_create/cmd_raw', MotorSpeed, queue_size=1)
        
        self.behaviours = []

    def addBehaviour(self, behaviour, priority=None):
        if behaviour not in self.behaviours:
            self.behaviours.append(behaviour)

            self.behaviours[-1].controller = self
            if priority != None:
                self.behaviours[-1].priority = priority

            # Sort by priority
            self.behaviours.sort(key=lambda x: x.priority, reverse=False)

    def isActiveBehaviour(self, ignored=[]):
        isActive = False
        for behaviour in self.behaviours:
            if behaviour.isActive and behaviour not in ignored:
                isActive = True
                break
        return isActive

    @staticmethod
    def callback(data, self):
        # Change current state
        with self.lock:
            if(isinstance(data, BatteryState)):
                self.state.battery = data
            elif(isinstance(data, Contact)):
                self.state.contact = data
            elif(isinstance(data, Odometry)):
                self.state.odom = data
                self.state._updateDistance()
                
    def executeAction(self, action, state):
        if isinstance(action, MotorAction):
            if self.lastMotorAction == None or (self.lastMotorAction.leftSpeed != action.leftSpeed or self.lastMotorAction.rightSpeed != action.rightSpeed):
            	# Send command to motor
            	msg = MotorSpeed()
                msg.header.stamp      =  rospy.Time.now()
                msg.header.frame_id   =  "cmd_raw"
            	msg.left              =  action.leftSpeed
            	msg.right             =  action.rightSpeed
            	self.rawPub.publish(msg)
        
            	self.lastMotorAction = action
            #else:
            #    # Do not send command, keep current state
            #    pass
        #elif isinstance(action, DockAction):
        #    if not self.isDocking and not state.contact.homeBase:
        #        self.isDocking = True
        #        self.dock()
        #    elif self.isDocking and state.homeBase:
        #        self.isDocking = False

    def spin(self):
        
        if self.rate is not None:
            r = rospy.Rate(int(self.rate))
        while not rospy.is_shutdown():

            # Get current state
            with self.lock:
                state = self.state
            if state == None:
                # Wait for the first state to become available
                continue

            isActionExecuted = False
            for behaviour in self.behaviours:
                action = behaviour.execute(state)
                if action != None and not isActionExecuted:
                    self.executeAction(action, state)
                    isActionExecuted = True

            if not isActionExecuted:
                action = MotorAction(0, 0)
                self.executeAction(action, state)
            
            if self.rate is not None:
                r.sleep()

class Spiral(Behaviour):
    
    SPEED = 250 # mm/s
    RADIUS = 100 # mm
    WHEELBASE_DISTANCE = 258 # mm
    
    def __init__(self, priority=0, controller=None):
        self.mode = None
        self.angleRef = 0
        Behaviour.__init__(self, priority, controller)
    
    def executeSpin(self, state):
        theta = state.getOdomAngle() - self.angleRef
        r = np.max(((Spiral.RADIUS / (2.0 * np.pi)) * theta, Spiral.RADIUS / (2.0 * np.pi)))
        k = (1.0 + (2 * r)/Spiral.WHEELBASE_DISTANCE) / ((2 * r)/Spiral.WHEELBASE_DISTANCE - 1.0)
        leftSpeed = 2.0 * Spiral.SPEED / (k + 1)
        rightSpeed = k * leftSpeed
        
        action = MotorAction(leftSpeed, rightSpeed)
        return action
    
    def execute(self, state):
        
        # Activation of behaviour
        if not self.isActive and not self.controller.isActiveBehaviour(ignored=[self]):
            self.activate()
            self.angleRef = state.getOdomAngle()
            self.mode = 'spin'
            rospy.logdebug("Spiral behaviour activated")
        elif self.isActive and self.controller.isActiveBehaviour(ignored=[self]):
            self.deactivate()
            self.mode = None
            rospy.logdebug("Spiral behaviour deactivated")
        elif self.isActive and (state.contact.bumpLeft or state.contact.bumpRight or state.contact.virtualWall):
            self.deactivate()
            self.mode = None
            rospy.logdebug("Spiral behaviour deactivated")
            
        # Execution branching depending on current mode
        action = None
        if self.mode == 'spin':
            action = self.executeSpin(state)
        return action
    
class Wander(Behaviour):
    
    SPEED = 250 # mm/s
    ROTATION_PERIOD = 5 # s (set to 0 to disable)
    
    def __init__(self, priority=0, controller=None):
        self.mode = None
        self.lastRotation = time.time()
        Behaviour.__init__(self, priority, controller)
    
    def executeForward(self, state):
        action = MotorAction(Wander.SPEED,Wander.SPEED)
        now = time.time()
        if Wander.ROTATION_PERIOD > 0.0 and now - self.lastRotation > Wander.ROTATION_PERIOD:
            self.angle = random.uniform((-math.pi/2.0), (math.pi/2.0))
            self.angleRef = state.getOdomAngle()
            self.mode = 'rotate'
            self.lastRotation = now
        return action
    
    def executeRotate(self, state):

        # Give rotate command
        if self.angle > 0.0:
            # Turn right
            action = MotorAction(Wander.SPEED,0)
        else:
            # Turn left
            action = MotorAction(0,Wander.SPEED)
            
        # Wait to get to proper angle
        rospy.logdebug("Current angle (%f desired): %f", self.angle, self.angleRef - state.getOdomAngle())
        if np.abs(self.angleRef - state.getOdomAngle()) > np.abs(self.angle):
            self.mode = 'move'
        return action
    
    def execute(self, state):

        # Activation of behaviour
        if not self.isActive and not self.controller.isActiveBehaviour(ignored=[self]):
            self.activate()
            self.lastRotation = time.time()
            self.mode = 'move'
            rospy.logdebug("Wander behaviour activated")
        elif self.isActive and self.controller.isActiveBehaviour(ignored=[self]):
            self.deactivate()
            self.mode = None
            rospy.logdebug("Wander behaviour deactivated")
            
        # Execution branching depending on current mode
        action = None
        if self.mode == 'rotate':
            action = self.executeRotate(state)
        elif self.mode == 'move':
            action = self.executeForward(state)
        return action
        
class AvoidObstacle(Behaviour):
    
    SPEED = 100 # mm/s
    DISTANCE = 35.0 # mm
    
    def __init__(self, priority=0, controller=None,
                 wheelDropEnabled=True, cliffEnabled=True, bumperEnabled=True):
        self.mode = None
        self.wheelDropEnabled = wheelDropEnabled
        self.cliffEnabled = cliffEnabled
        self.bumperEnabled = bumperEnabled
        
        Behaviour.__init__(self, priority, controller)
    
    def executeStop(self, state):
        # Brake
        return MotorAction(0,0)

    def executeRotate(self, state):

        # Give rotate command
        if self.angle > 0.0:
            action = MotorAction(AvoidObstacle.SPEED,-AvoidObstacle.SPEED)
        else:
            action = MotorAction(-AvoidObstacle.SPEED,AvoidObstacle.SPEED)
            
        # Wait to get to proper angle
        rospy.logdebug("Current angle (%f desired): %f", self.angle, self.angleRef - state.getOdomAngle())
        if np.abs(self.angleRef - state.getOdomAngle()) > np.abs(self.angle):
            self.mode = None
        return action
    
    def executeMove(self, state):
        
        # Give move command
        if self.distance > 0.0:
            action = MotorAction(AvoidObstacle.SPEED,AvoidObstacle.SPEED)
        else:
            action = MotorAction(-AvoidObstacle.SPEED,-AvoidObstacle.SPEED)
        
        # Wait to get to proper distance
        rospy.logdebug("Current distance (%f desired): %f", self.distance, self.distanceRef - state.getOdomDistance())
        if np.abs(self.distanceRef - state.getOdomDistance()) > np.abs(self.distance):
            if self.mode == 'move':
                self.mode = None
            elif self.mode == 'avoid':
                self.mode = 'rotate'
        return action

    def execute(self, state):

        # Wheel drop detection
        if self.wheelDropEnabled and (state.contact.wheeldropCaster or state.contact.wheeldropLeft or state.contact.wheeldropRight):
            # Brake
            self.mode = 'stop'
        # Cliff detection
        elif self.cliffEnabled and (state.contact.cliffLeft or state.contact.cliffFrontLeft):
            # Turn right
            self.angle = random.uniform(math.pi*0.44, math.pi*1.31)
            self.angleRef = state.getOdomAngle()
            self.mode = 'rotate'
        elif self.cliffEnabled and (state.contact.cliffFrontRight or state.contact.cliffRight):
            # Turn left
            self.angle = -random.uniform(math.pi*0.44, math.pi*1.31)
            self.angleRef = state.getOdomAngle()
            self.mode = 'rotate'
        # Bumper detection
        elif self.bumperEnabled and state.contact.bumpLeft:
            # Turn right
            self.angle = random.uniform(math.pi*0.44, math.pi*1.31)
            self.angleRef = state.getOdomAngle()
            self.distance = -AvoidObstacle.DISTANCE
            self.distanceRef = state.getOdomDistance()
            self.mode = 'avoid'
        elif self.bumperEnabled and state.contact.bumpRight:
            # Turn left
            self.angle = -random.uniform(math.pi*0.44, math.pi*1.31)
            self.angleRef = state.getOdomAngle()
            self.distance = -AvoidObstacle.DISTANCE
            self.distanceRef = state.getOdomDistance()
            self.mode = 'avoid'
        elif self.bumperEnabled and state.contact.virtualWall:
            # Turn either left or right
            angle = random.uniform(math.pi*0.44, math.pi*1.31)
            if random.random() > 0.5:
                # Turn right
                self.angle = angle
            else:
                # Turn left
                self.angle = -angle
            self.angleRef = state.getOdomAngle()
            self.distance = -AvoidObstacle.DISTANCE
            self.distanceRef = state.getOdomDistance()
            self.mode = 'avoid'

        # Activation of the behaviour
        if not self.isActive and self.mode != None:
            self.activate()
            rospy.logdebug("Avoid-obstacle behaviour activated")
        elif self.isActive and self.mode == None:
            self.deactivate()
            rospy.logdebug("Avoid-obstacle behaviour deactivated")
            
        # Execution branching depending on current mode
        action = None
        if self.mode == 'stop':
            action = self.executeStop(state)
        elif self.mode == 'rotate':
            action = self.executeRotate(state)
        elif self.mode == 'move' or self.mode == 'avoid':
            action = self.executeMove(state)
        return action


class FollowWall(Behaviour):

    SPEED = 50 # mm/s
    WALL_DISTANCE_THRESHOLD = 175
    WALL_DETECT_THRESHOLD = 25
    SENSITIVITY = 20.0
 
    def __init__(self, priority=0, controller=None):
        self.mode = None
        Behaviour.__init__(self, priority, controller)
    
    def executeFollow(self, state):
        diff = state.contact.wallSignal - FollowWall.WALL_DISTANCE_THRESHOLD
        if diff > 0:
            # Forward with left turn (too close)
            action = MotorAction(FollowWall.SPEED, FollowWall.SPEED + FollowWall.SENSITIVITY)
        elif diff < 0:
            # Forward with right turn (too far)
            action = MotorAction(FollowWall.SPEED + FollowWall.SENSITIVITY, FollowWall.SPEED)
        else:
            # Continue straight
            action = MotorAction(FollowWall.SPEED, FollowWall.SPEED)
        return action
        
    def execute(self, state):
        
        # Activation of behaviour
        if not self.isActive and state.contact.wallSignal >= FollowWall.WALL_DETECT_THRESHOLD:
            self.activate()
            self.mode = 'follow'
            rospy.logdebug("Follow-wall behaviour activated")
        elif self.isActive and state.contact.wallSignal < FollowWall.WALL_DETECT_THRESHOLD:
            self.deactivate()
            self.mode = None
            rospy.logdebug("Follow-wall behaviour deactivated")
            
        # Execution branching depending on current mode
        action = None
        if self.mode == 'follow':
            action = self.executeFollow(state)
        return action

class Charge(Behaviour):
    
    MAX_CHARGING_TIME = 5 # s 
    BATTERY_FULL_THRESHOLD = 90 # % charged
     
    def __init__(self, priority=0, controller=None):
        self.mode = None
        self.chargingStartTime = time.time()
        Behaviour.__init__(self, priority, controller)

    def executeCharge(self, state):
        action = MotorAction(0, 0)
        return action

    def execute(self, state):
          
        # Activation of behaviour
        chargingTime = time.time() - self.chargingStartTime
        batteryLevel = float(state.battery.batteryCharge) / state.battery.batteryCapacity * 100.0
        if not self.isActive : #and (state.homeBase or state.internalCharger):
            self.activate()
            self.mode = 'charge'
            self.chargingStartTime = time.time()
            rospy.logdebug("Charge behaviour activated")
        elif self.isActive and ( batteryLevel >= Charge.BATTERY_FULL_THRESHOLD or chargingTime >= Charge.MAX_CHARGING_TIME):
            self.deactivate()
            self.mode = None
            rospy.logdebug("Charge behaviour deactivated")
            
        action = None
        if self.mode == 'charge':
            action = self.executeCharge(state)
        return action
        
#class Undock(Behaviour):
#    
#    SPEED = 50 # mm/s
#    DISTANCE = 500 # mm
#    BATTERY_FULL_THRESHOLD = 90 # % charged
#    
#    def __init__(self, priority=0, controller=None):
#     
#        self.mode = None
#        self.clear = False
#        Behaviour.__init__(self, priority, controller)
#    
#    def executeRotate(self, state):
#
#        # Give rotate command
#        if self.angle > 0.0:
#            action = MotorAction(AvoidObstacle.SPEED,-AvoidObstacle.SPEED)
#        else:
#            action = MotorAction(-AvoidObstacle.SPEED,AvoidObstacle.SPEED)
#            
#        # Wait to get to proper angle
#        rospy.logdebug("Current state.odom.angle (%f desired): %f", self.angle, self.angleRef - state.angle)
#        if np.abs(self.angleRef - state.odom.angle) > np.abs(self.angle):
#            self.mode = None
#            self.clear = True
#        return action
#    
#    def executeMove(self, state):
#        
#        # Give move command
#        if self.distance > 0.0:
#            action = MotorAction(AvoidObstacle.SPEED,AvoidObstacle.SPEED)
#        else:
#            action = MotorAction(-AvoidObstacle.SPEED,-AvoidObstacle.SPEED)
#        
#        # Wait to get to proper distance
#        rospy.logdebug("Current state.odom.distance (%f desired): %f", self.distance, self.distanceRef - state.distance)
#        if np.abs(self.distanceRef - state.odom.distance) > np.abs(self.distance):
#            self.mode = 'rotate'
#        return action
#        
#    def execute(self, state):
#          
#        # Activation of behaviour
#        batteryLevel = float(state.battery.batteryCharge) / state.battery.batteryCapacity * 100.0
#        if not self.isActive : # and state.homeBase and batteryLevel >= Undock.BATTERY_FULL_THRESHOLD:
#            self.activate()
#            self.angle = 180
#            self.angleRef = state.odom.angle
#            self.distance = -Undock.DISTANCE
#            self.distanceRef = state.odom.distance
#            self.mode = 'move'
#            self.clear = False
#            rospy.logdebug("Undock behaviour activated")
#        elif self.isActive and self.clear: # and state.homeBase == False
#            self.deactivate()
#            self.mode = None
#            rospy.logdebug("Undock behaviour deactivated")
#            
#        action = None
#        if self.mode == 'move':
#            action = self.executeMove(state)
#        elif self.mode == 'rotate':
#            action = self.executeRotate(state)
#        return action
#   
#class AutoDock(Behaviour):
#     
#    BATTERY_LOW_THRESHOLD = 90 # % charged
#    NOTHING = 255
#     
#    def __init__(self, priority=0, controller=None):
#        self.mode = None
#        Behaviour.__init__(self, priority, controller)
#        
#    def execute(self, state):
#            
#            # Activation of behaviour
#            batteryLevel = float(state.battery.batteryCharge) / state.battery.batteryCapacity * 100.0
#            if not self.isActive and state.infraredByte != AutoDock.NOTHING and batteryLevel <= AutoDock.BATTERY_LOW_THRESHOLD:
#                self.activate()
#                self.mode = 'autodocking'
#                rospy.logdebug("AutoDock behaviour activated")
#            elif self.isActive and state.homeBase == True:
#                self.deactivate()
#                self.mode = None
#                rospy.logdebug("AutoDock behaviour deactivated")
#                
#            action = None
#            if self.mode == 'autodocking':
#                action = DockAction()
#                
#            return action
#        
#class ManualDock(Behaviour):
#     
#    NOTHING = 255
#    RED_BUOY = 248
#    GREEN_BUOY = 244
#    FORCE_FIELD = 242
#    RED_BUOY_GREEN_BUOY = 252
#    RED_BUOY_FORCE_FIELD = 250
#    GREEN_BUOY_FORCE_FIELD = 246
#    RED_BUOY_GREEN_BUOY_FORCE_FIELD = 254
#     
#    def __init__(self, priority=0, controller=None):
#     
#        # Accumulator for IR signals
#        buffer = np.zeros(10)
#        bufferIdx = 0
#        bufferLen = 0
#        
#        self.mode = None
#        Behaviour.__init__(self, priority, controller)
#
#    def onBeamTransitionDetected(self, initial, final):
#        if initial == NOTHING and final == GREEN_BUOY:
#            # Turn left 45
#            pass
#        elif initial == NOTHING and final == RED_BUOY:
#            # Turn right 45
#            pass
#        elif initial == GREEN_BUOY and final == NOTHING:
#            pass
#        elif initial == RED_BUOY and final == NOTHING:
#            pass
#        elif initial == GREEN_BUOY and final == RED_BUOY_GREEN_BUOY:
#            pass
#        elif initial == RED_BUOY and final == RED_BUOY_GREEN_BUOY:
#            pass
#        elif initial == RED_BUOY_GREEN_BUOY and final == GREEN_BUOY:
#            pass
#        elif initial == RED_BUOY_GREEN_BUOY and final == RED_BUOY: 
#            pass
#         
#    def executeHoming(self, state):
#         
#        # Get the dock beam configuration
#        value = None
#        if state.infraredByte == RED_BUOY or state.infraredByte == RED_BUOY_FORCE_FIELD:
#            value = 1
#        elif state.infraredByte == GREEN_BUOY or state.infraredByte == GREEN_BUOY_FORCE_FIELD:
#            value = -1
#        elif state.infraredByte == RED_BUOY_GREEN_BUOY or state.infraredByte == RED_BUOY_GREEN_BUOY_FORCE_FIELD:
#            value = 0
#        elif state.infraredByte == NOTHING:
#            value = -10
#             
#        # Accumulate value to buffer (for averaging)
#        if value != None:
#            buffer[bufferIdx] = value
#            bufferIdx = bufferIdx + 1
#            if bufferIdx >= len(buffer):
#                bufferIdx = 0
#            if bufferLen < len(buffer):
#                bufferLen = bufferLen + 1
#             
#            avgValue = np.mean(buffer[0:bufferLen])
#             
#             
#            if avgValue == 0:
#                # Forward
#                pass
#         
#        # If red, then red+green = turn right
#        # If green, then red+green = turn left
#        # If red,green = forward
#        # if red,green then red = turn left
#        # if red,green then green = turn right
#        pass
#     
#    def execute(self, state):
#        
#        # Activation of behaviour
#        if not self.isActive and state.infraredByte != Dock.NOTHING:
#            self.activate()
#            self.mode = 'homing'
#        elif self.isActive and state.homeBase == True:
#            self.deactivate()
#            self.mode = None
#        
#        action = None
#        if mode == 'homing':
#            action = self.executeHoming(state)
#        return action
