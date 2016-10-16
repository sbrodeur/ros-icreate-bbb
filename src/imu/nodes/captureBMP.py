#!/usr/bin/env python

# Copyright (c) 2016, Simon Brodeur & Simon Carrier
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

import sys,io
import array

import rospy

from sensor_msgs.msg import FluidPressure, Temperature
from std_msgs.msg import Header

class CaptureBMP():


    def __init__(self,  deviceLoc, tempProbe, presProbe, rate ):

        self.deviceLoc = str(deviceLoc)
        self.tempProbe = str(tempProbe)
        self.presProbe = str(presProbe)
        self.rate = rate


        #Open io streams
        self.presFile = io.open(self.deviceLoc + self.presProbe, "rb") 

        self.tempFile = io.open(self.deviceLoc + self.tempProbe, "rb")

        if not ( self.tempFile.readable() and self.presFile.readable()):
            rospy.logerr('Unable to open BMP stream!')
            exit(2)

        # Create publisher
        outputTemp = rospy.get_param('~outputTemp', '/imu/temp')
        self.pubTemp = rospy.Publisher(outputTemp, Temperature, queue_size=5)

        outputPres = rospy.get_param('~outputPres', '/imu/pres')
        self.pubPres = rospy.Publisher(outputPres, FluidPressure, queue_size=5)


    def close(self):
        self.tempFile.close()
        self.presFile.close()

    def spin(self):
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():

            currentTemperature = 0
            currentPressure = 0
            try:
                self.tempFile.seek(0)
                self.presFile.seek(0)
                currentTemperature = self.tempFile.read().replace('\n','')
                currentPressure = self.presFile.read().replace('\n','')
            except IOError:
                rospy.logerr('Unable to read BMP stream!')

            # Publish temperature data
            msgT = Temperature()
            msgT.header.stamp = rospy.Time.now()
            msgT.temperature = float(currentTemperature)/10.0
            self.pubTemp.publish(msgT)
            
            # Publish temperature data
            msgP = FluidPressure()
            msgP.header.stamp = rospy.Time.now()
            msgP.fluid_pressure = float(currentPressure)
            self.pubPres.publish(msgP)
            
            r.sleep()


if __name__ == '__main__':

    capture = None
    try:
        rospy.init_node('captureBMP', log_level=rospy.INFO, anonymous=True)
        deviceLoc = rospy.get_param('~deviceLocation', "/sys/bus/i2c/drivers/bmp085/2-0077")
        tempProbe = rospy.get_param('~tempProbe', "/temp0_input")
        presProbe = rospy.get_param('~presProbe', "/pressure0_input")
        rate = rospy.get_param('~rateHz', 1)

        capture = CaptureBMP( deviceLoc, tempProbe, presProbe, rate)
        capture.spin()
    except rospy.ROSInterruptException: pass

    if capture != None:
        capture.close()
