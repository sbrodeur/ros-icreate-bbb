import sys, os, struct, array
import numpy as np
import rosbag
import math
import matplotlib.pyplot as plt
from tabulate import tabulate

from optparse import OptionParser

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
    
    def printHistograms(self):
        for topicStat in self.mTopicsStatList:
            topicStat.printHistogram()

    def printTabulatedResults(self, toFile="" ):
        wCatNames = np.array([["Topic name", "Mean", "Average Rate", "Std Deviation", "Nb of Messages", "Est dropped %" ]])
        wTable = np.array([[]])
        
        for topicStat in self.mTopicsStatList:
            if wTable.size == 0 :
                wTable = topicStat.getStatsTable()
            else :
                wTable = np.concatenate((wTable, topicStat.getStatsTable()), axis=0)

        wTable = wTable[np.argsort(wTable[:,0])]
        wTable = np.concatenate((wCatNames, wTable), axis=0)
        
        if not toFile :
            print tabulate(wTable, headers="firstrow", tablefmt='grid')
        else : 
            f1=open( toFile , 'w')
            print >>f1, tabulate(wTable, headers="firstrow", tablefmt='grid')
            f1.close()

#Generates and keeps the statistics of a single topic
class TopicStatistics :

    def __init__(self, iTopicName, iThres=1.1):
        
        #Data vars
        self.mTopicName = str(iTopicName)
        self.mTimestampList = np.array([])
        self.mTimesBetweenMsgs = np.array([])
        self.mThres = iThres           #threshold to indicate dropped message

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
                if(tDur > self.mMean + self.mMean*self.mThres):
                    self.mNumOfDroppedMsgs += 1
            wHistDomain = self.mMean + 5.0*self.mStdDeviation 
            self.mHistogram = np.histogram(self.mTimesBetweenMsgs, bins=50)    

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
         

        return wPrintMsg 

    def getStatsTable(self):
        return np.array([[str(self.mTopicName),  self.mMean, self.mAverageRate, self.mStdDeviation, self.mTimestampList.size, float(self.mNumOfDroppedMsgs)/float(self.mTimestampList.size)*100.0]])

    def printHistogram(self):
        
        plt.hist(self.mTimesBetweenMsgs, bins=300)  
        plt.axvline(self.mMean, color='k')
        plt.axvline(self.mMean - self.mStdDeviation, color='r')
        plt.axvline(self.mMean + self.mStdDeviation, color='r')
        
        plt.title("Histogram for " + str(self.mTopicName))
        plt.show()

if __name__ == '__main__':

       
    parser = OptionParser()
    parser.add_option("-i", "--input", dest="filename",
             help="Location of Rosbag to analyse", default=False)

    parser.add_option("-g", action="store_true", dest="showGraph",
                        help="Shows histogram of each topic", default=False)
    
    parser.add_option("-o",  dest="outputDest", help="Output stats file", default="")
    parser.add_option("-s", action="store_true", dest="sameOutputDest", 
             help="Output stats file into the same location as the rosbag", default=False)
     
    (options, args) = parser.parse_args()

    if not options.filename:
        parser.error("Please specify input rosbag with -i ")

    if  options.outputDest and options.sameOutputDest:
        parser.error("-o and -s are mutually exclusive ")


    rosStats = RosbagStatistics(options.filename)
    rosStats.generateStats()

    if options.outputDest :
        rosStats.printTabulatedResults(toFile=options.outputDest)
    elif options.sameOutputDest:
        rosStats.printTabulatedResults(toFile=options.filename + ".stat") 
    else :
        rosStats.printTabulatedResults()
    
    if options.showGraph :
        rosStats.printHistograms()





