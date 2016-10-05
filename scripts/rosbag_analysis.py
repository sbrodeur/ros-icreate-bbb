import sys, os, struct, array
import numpy as np
import rosbag
import math
import matplotlib.pyplot as plt


import rospy

#Manages the statistics of the whole rosbags 
#Fetches the rosbag and its topics
class RosbagStatistics :
    
    def __init__( self, iRosbagName ):
        
        self.mRosbag = rosbag.Bag(iRosbagName)
        
       
        #Grab list of topics
        wTopicsNamesList = self.mRosbag.get_type_and_topic_info()[1].keys()
        
        #Generate Topic statisitics objects
        self.mTopicsStatList = np.array([])
        for topicName in wTopicsNamesList:
            self.mTopicsStatList = np.append(self.mTopicsStatList, TopicStatistics(topicName))

        for topic, msg, t in self.mRosbag.read_messages(): 
            for tTopicStat in self.mTopicsStatList :
                if tTopicStat.mTopicName == topic :
                    tTopicStat.addStamps(t)
                    break
        self.mRosbag.close()

    def generateStats(self):
        for topicStat in self.mTopicsStatList:
            topicStat.generateStatistics()

    def printResults(self):
        for topicStat in self.mTopicsStatList:
            print topicStat.printStats()


#Generates and keeps the statistics of a single topic
class TopicStatistics :

    def __init__(self, iTopicName, iThres=3.0):
        
        #Data vars
        self.mTopicName = iTopicName
        self.mTimestampList = np.array([])
        self.mTimesBetweenMsgs = np.array([])
        self.mSigThres = iThres           #threshold in sigma to indicate dropped message

        #Statistics vars
        self.mAverageRate = 0
        self.mMean = 0
        self.mStdDeviation = 0
        self.mVariance = 0
        self.mNumOfDroppedMsgs = 0
        self.mHistogram = np.histogram(np.array([]))

    def addStamps(self, time):
        self.mTimestampList = np.append(self.mTimestampList, time)
    

    def generateStatistics(self):
        if self.sanityCheck() == "Good" :
            for i in range(0,len(self.mTimestampList)-1):
                tDuration = self.mTimestampList[i+1] - self.mTimestampList[i]
                tDurationInSec = float(tDuration.secs) + tDuration.nsecs*(10**-9) 
                self.mTimesBetweenMsgs = np.append(self.mTimesBetweenMsgs, tDurationInSec)
                 
            self.mMean = sum(self.mTimesBetweenMsgs)/float(len(self.mTimesBetweenMsgs))
            self.mVariance = sum((self.mTimesBetweenMsgs - self.mMean)**2)/float(len(self.mTimesBetweenMsgs))
            self.mStdDeviation = math.sqrt(self.mVariance)
            self.mAverageRate = 1.0/self.mMean
            for tDur in self.mTimesBetweenMsgs :
                if(tDur > self.mMean + self.mStdDeviation*self.mSigThres):
                    self.mNumOfDroppedMsgs += 1
            wHistDomain = self.mMean + 5.0*self.mStdDeviation 
            self.mHistogram = np.histogram(self.mTimesBetweenMsgs, bins=10)    

    def sanityCheck(self):
        if self.mTimestampList.size == 0:
            return "No time stamps!"
        elif not self.mTopicName :
            return "No Topic name!"
        else:
            return "Good"
    
    def printStats(self) :
        wPrintMsg  = "======================================\n"
        wPrintMsg += "TOPIC : " + str(self.mTopicName) + "\n"
        wPrintMsg += "Mean : " + str(self.mMean) + " s" + "\n"
        wPrintMsg += "Average Rate : " + str(self.mAverageRate) +" msgs/s"+ "\n"
        wPrintMsg += "Std Deviation : " + str(self.mStdDeviation) + "\n"
        wPrintMsg += "Number of Msgs : " + str(len(self.mTimestampList)) + "\n"
        wPrintMsg += "Est Dropped % : " + str(float(self.mNumOfDroppedMsgs)/float(self.mTimestampList.size)*100.0) + "\n"
        wPrintMsg += "======================================\n\n"
         
        plt.hist(self.mTimesBetweenMsgs, bins=10)  
        plt.axvline(self.mMean, color='k')
        plt.axvline(self.mMean - self.mStdDeviation, color='r')
        plt.axvline(self.mMean + self.mStdDeviation, color='r')
        
        plt.title("Histogram for " + str(self.mTopicName))
        plt.show()

        return wPrintMsg 

if __name__ == '__main__':

    rosbagLoc = ""
    if (len(sys.argv) == 1):
        print "Usage: Specify rosbag location"
        sys.exit(2)
    elif (len(sys.argv) == 2):
        rosbagLoc = sys.argv[1]
        
    rosStats = RosbagStatistics(rosbagLoc)
    rosStats.generateStats()
    rosStats.printResults()





