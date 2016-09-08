#!/usr/bin/python
import roslib
import rospy
from time import sleep
from irobot import Create
from threading import Thread
from math import sin,cos,pi
from datetime import datetime

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from sensor_msgs.msg import BatteryState

from irobot_create.msg import SensorPacket, MotorControl
from irobot_create.srv import *

class SafetyRestriction(object):
    NONE = 0
    DONTMOVE = 1
    TURNONLY = 2
    BACKONLY = 3

class CreateDriver:
    
    def __init__(self):
        port = rospy.get_param('~port', "/dev/ttyUSB0")
        self.autodock = rospy.get_param('~autodock', 0.0)
        self.name = rospy.get_param('~name', "irobot_create")
        self.chargingEnabled = rospy.get_param('~charging', True)
        self.safetyEnabled = rospy.get_param('~safety', True)
        self.create = Create(port)
        self.packetPub = rospy.Publisher(self.name + '/sensorPacket', SensorPacket, queue_size=1)
        self.battPub = rospy.Publisher(self.name + '/battery', BatteryState, queue_size=1)
        self.odomPub = rospy.Publisher(self.name + '/odom',Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()
        self.fields = ['wheeldropCaster','wheeldropLeft','wheeldropRight','bumpLeft','bumpRight','wall','cliffLeft','cliffFrontLeft','cliffFrontRight','cliffRight','virtualWall','infraredByte','advance','play','distance','angle','chargingState','voltage','current','batteryTemperature','batteryCharge','batteryCapacity','wallSignal','cliffLeftSignal','cliffFrontLeftSignal','cliffFrontRightSignal','cliffRightSignal','homeBase','internalCharger','songNumber','songPlaying','requestedRightVelocity','requestedLeftVelocity']
        self.then = datetime.now() 
        self.x = 0
        self.y = 0
        self.th = 0
        self.create.update = self.sense
        self.docking = False
        self.safetyRestriction = SafetyRestriction.NONE
        self.isProblem = False
        self.lastBatteryCheck = None

    def start(self):
        self.create.start()
        self.then = datetime.now() 

    def stop(self):
        self.create.stop()

    def applySafety(self):
        
        isProblem = False
        
        # Cliff detection
        if self.create.cliffLeft or self.create.cliffFrontLeft or self.create.cliffFrontRight or self.create.cliffRight :
            self.safetyRestriction = SafetyRestriction.BACKONLY
            rospy.logdebug("Safety activated: Cliff detection")
            rospy.logdebug("Allowing back action only")
            isProblem = True
            
        # Bumper detection
        if self.create.bumpLeft or self.create.bumpRight or self.create.virtualWall:
            self.safetyRestriction = SafetyRestriction.BACKONLY
            rospy.logdebug("Safety activated: Collision detection with bumper")
            rospy.logdebug("Allowing back action only")
            isProblem = True
            
        # Wheel drop detection
        if self.create.wheeldropCaster or self.create.wheeldropLeft or self.create.wheeldropRight:
            self.safetyRestriction = SafetyRestriction.DONTMOVE
            rospy.logdebug("Safety activated: Wheel drop detection")
            rospy.logdebug("Allowing no action")
            isProblem = True
            
        if not isProblem:
            self.safetyRestriction = SafetyRestriction.NONE
            
        return isProblem

    def checkBattery(self):
        chargeRemaining = float(self.create.batteryCharge) / self.create.batteryCapacity * 100.0 
        level = 255 - int(chargeRemaining / 100.0 * 255)
        self.create.leds(0,0,level,255)
        if self.create.voltage < 14.0 or chargeRemaining < 10.0:
            rospy.loginfo("Battery level is low!")
            self.create.beep()
        self.lastBatteryCheck = datetime.now()

    def sense(self):
        
        if self.safetyEnabled:
            isProblem = self.applySafety()
            if not self.isProblem and isProblem:
                # Apply brake on problem detection
                self.create.brake()
                rospy.logdebug("Safety activated: brake enabled")
                self.isProblem = True
            elif self.isProblem and not isProblem:
                rospy.logdebug("Safety deactivated")
                self.isProblem = False
        
        now = datetime.now()
        if self.lastBatteryCheck == None or (now - self.lastBatteryCheck).seconds > 30:
            self.checkBattery()
        
        if self.chargingEnabled:
            if self.create.mode == 'on' and (self.create.homeBase or self.create.internalCharger):
                # Activate passive mode to allow charging
                rospy.loginfo("Sleeping now (passive mode)")
                self.create.sleep()
            elif self.create.mode == 'sleep' and not (self.create.homeBase or self.create.internalCharger):
                # Reactivate active mode
                rospy.loginfo("Resuming now (active mode)")
                self.create.resume()
                self.checkBattery()
                
        now = datetime.now()
        elapsed = now - self.then
        self.then = now
        elapsed = float(elapsed.seconds) + elapsed.microseconds/1000000.
        d = self.create.d_distance / 1000.
        th = self.create.d_angle*pi/180
        dx = d / elapsed
        dth = th / elapsed

        if (d != 0):
            x = cos(th)*d
            y = -sin(th)*d
            self.x = self.x + (cos(self.th)*x - sin(self.th)*y)
            self.y = self.y + (sin(self.th)*x + cos(self.th)*y)

        if (th != 0):
            self.th = self.th + th

        quaternion = Quaternion()
        quaternion.x = 0.0 
        quaternion.y = 0.0
        quaternion.z = sin(self.th/2)
        quaternion.w = cos(self.th/2)

        self.odomBroadcaster.sendTransform(
            (self.x, self.y, 0), 
            (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
            rospy.Time.now(),
            "base_link",
            "odom"
            )

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0
        odom.pose.pose.orientation = quaternion

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = dx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = dth

        self.odomPub.publish(odom)

        packet = SensorPacket()
        for field in self.fields:
            packet.__setattr__(field,self.create.__getattr__(field))
        self.packetPub.publish(packet)

        battPacket = BatteryState()
        battPacket.voltage = self.create.__getattr__('voltage')
        battPacket.current = self.create.__getattr__('current')
        battPacket.charge  = self.create.__getattr__('batteryCharge')
        battPacket.capacity = self.create.__getattr__('batteryCapacity')
        #battPacket.design_capacity = self.create.__getattr__('voltage')
        battPacket.percentage = battPacket.charge/battPacket.capacity
        #battPacket.power_supply_status = self.create.__getattr__('voltage')
        #battPacket.power_supply_health = self.create.__getattr__('voltage')
        #battPacket.power_supply_technology = self.create.__getattr__('voltage')
        battPacket.present = True
        
        self.battPub.publish(battPacket)

        charge_level = float(packet.batteryCharge) / float(packet.batteryCapacity)
        if (self.docking and packet.homeBase and charge_level > 0.95):
            self.docking = False
            self.create.reset()
            rospy.sleep(1)
            self.create.start()
        elif (not self.docking and charge_level < self.autodock):
            self.create.demo(1)
            self.docking = True

    def brake(self,req):
        if (req.brake):
            self.create.brake()
        return BrakeResponse(True)

    def circle(self,req):
        if (req.clear):
            self.create.clear()
            
        commandIgnored=False
        if self.safetyRestriction == SafetyRestriction.NONE:
            self.create.forwardTurn(req.speed,req.radius)
        else:
            commandIgnored=True
        return CircleResponse(not commandIgnored)

    def demo(self,req):
        self.create.demo(req.demo)
        return DemoResponse(True)

    def leds(self,req):
        self.create.leds(req.advance,req.play,req.color,req.intensity)
        return LedsResponse(True)

    def tank(self,req):
        if (req.clear):
            self.create.clear()
            
        commandIgnored=False
        if self.safetyRestriction == SafetyRestriction.NONE:
            self.create.tank(req.left,req.right)
        elif self.safetyRestriction == SafetyRestriction.TURNONLY:
            if req.left == 0 or req.right == 0:
                self.create.tank(req.left,req.right)
            else:  
                commandIgnored = True
        elif self.safetyRestriction == SafetyRestriction.BACKONLY:
            if req.left < 0 or req.right < 0:
                self.create.tank(req.left,req.right)
            else:  
                commandIgnored = True
        else:
            commandIgnored=True
            
        return TankResponse(not commandIgnored)

    def turn(self,req):
        if (req.clear):
            self.create.clear()
            
        commandIgnored=False
        if self.safetyRestriction == SafetyRestriction.NONE or self.safetyRestriction == SafetyRestriction.TURNONLY:
            self.create.turn(req.turn)
        else:
            commandIgnored=True
        return TurnResponse(not commandIgnored)

    def dock(self,req):
        self.create.demo(1)
        self.docking = True
        return DockResponse(True)

    def twist(self,req):
        x = req.linear.x*1000.
        th = req.angular.z
        if (x == 0):
            if self.safetyRestriction == SafetyRestriction.NONE or self.safetyRestriction == SafetyRestriction.TURNONLY:
                th = th*180/pi
                speed = (8*pi*th)/9
                self.create.left(int(speed))
        elif (th == 0):
            if self.safetyRestriction == SafetyRestriction.NONE:
                x = int(x)
                self.create.tank(x,x)
            elif self.safetyRestriction == SafetyRestriction.BACKONLY:
                if x < 0:
                    x = int(x)
                    self.create.tank(x,x)
        else:
            if self.safetyRestriction == SafetyRestriction.NONE:
                self.create.forwardTurn(int(x),int(x/th))

    def vel(self,req):
        commandIgnored=False
        if self.safetyRestriction == SafetyRestriction.NONE:
            self.create.tank(req.left,req.right)
        elif self.safetyRestriction == SafetyRestriction.TURNONLY:
            if req.left == 0 or req.right == 0:
                self.create.tank(req.left,req.right)
            else:  
                commandIgnored = True
        elif self.safetyRestriction == SafetyRestriction.BACKONLY:
            if req.left < 0 or req.right < 0:
                self.create.tank(req.left,req.right)
            else:  
                commandIgnored = True
        else:
            commandIgnored=True

if __name__ == '__main__':
    rospy.init_node('irobot_create')
    driver = CreateDriver()
    
    rospy.Service(driver.name + '/brake',Brake,driver.brake)
    rospy.Service(driver.name + '/circle',Circle,driver.circle)
    rospy.Service(driver.name + '/demo',Demo,driver.demo)
    rospy.Service(driver.name + '/leds',Leds,driver.leds)
    rospy.Service(driver.name + '/tank',Tank,driver.tank)
    rospy.Service(driver.name + '/turn',Turn,driver.turn)
    rospy.Service(driver.name + '/dock',Dock,driver.dock)

    rospy.Subscriber(driver.name + "/cmd_twist", Twist, driver.twist)
    rospy.Subscriber(driver.name + "/cmd_raw", MotorControl, driver.vel)

    sleep(1)
    driver.start()
    sleep(1)

    rospy.spin()
    driver.stop()
