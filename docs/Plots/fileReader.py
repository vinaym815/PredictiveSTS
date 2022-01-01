import numpy as np
import re

class file:
    def __init__(self, columnNames, fileName, nHeaderLines, delimiter="\t", regrexHeader = r"\w+"):
        self.fileData = np.genfromtxt(fileName, delimiter=delimiter, skip_header=nHeaderLines+1)
        self.dataIdx = {}
        self.fileName = fileName
        
        with open(fileName) as infile:
            content = infile.readlines()
            row = content[nHeaderLines]
            headers = row.strip("\n").split(delimiter)
        
        for j, headerRaw in enumerate(headers):
            if (headerRaw=="time"):
                self.dataIdx.update({headerRaw : j})
            else:
                x = re.findall(regrexHeader, headerRaw)
                if x: 
                    if (x[0] in columnNames):
                        self.dataIdx.update({x[0] : j})
        return None
    
    def getColumn(self, colName):
        colNumber = self.dataIdx[colName]
        return self.fileData[:, colNumber]
    
    def getNumRows(self):
        return self.fileData.shape[0]

def computeMotionTimeRange(labels, thresholds, motionFileName, numHeaderLines, nPointMovingAverage):
    fileData = file(labels, motionFileName, numHeaderLines)
    time = fileData.getColumn(labels[0])
    timeD = np.diff(time)
    jointD = np.diff(fileData.getColumn(labels[1]))
    
    jointVel = np.divide(jointD, timeD)
    
    #smoothing with rolling average
    kernel_size = nPointMovingAverage
    kernel = np.ones(kernel_size) / kernel_size
    jointVel = np.convolve(jointVel, kernel, mode='same')
    
    jointVelRec = np.abs(jointVel)
    
    startInd = np.argmax(jointVelRec>=thresholds[0])
    endInd = jointVelRec.shape[0] - np.argmax(jointVelRec[::-1]>=thresholds[1])
    return time[startInd], time[endInd]

def computePhaseTimes(force_file, motion_file):
    ind = np.argmax(force_file.getColumn("seatConstraint_ground_Fy") == 0)
    e1 = force_file.getColumn("time")[ind]
    
    ind = np.argmax(motion_file.getColumn("hip_flexion"))
    e2 = motion_file.getColumn("time")[ind]
    
    return (e1, e2)