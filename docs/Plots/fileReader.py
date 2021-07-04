import numpy as np
import re

class file:
    def __init__(self, columnNames, fileName, nHeaderLines, delimiter="\t", regrexHeader = r"\w+"):
        self.fileData = np.genfromtxt(fileName, delimiter=delimiter, skip_header=nHeaderLines+1)
        self.dataIdx = {}
        
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
    
    
def computePhaseTimes(force_file, motion_file, com_file):
    ind = np.argmax(force_file.getColumn("seatConstraint_ground_Fy") == 0)
    e1 = force_file.getColumn("time")[ind]
    
    ind = np.argmax(motion_file.getColumn("hip_flexion"))
    e2 = motion_file.getColumn("time")[ind]
    
    ind = np.argmax(com_file.getColumn("com_vel_x")<0)
    e3 = com_file.getColumn("time")[ind]
    
    return (e1, e2, e3)