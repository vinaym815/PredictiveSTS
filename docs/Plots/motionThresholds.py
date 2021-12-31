#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file
from fileReader import computeMotionTimeRange


# In[2]:


plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": False,
    "font.sans-serif": ["CMU Sans Serif"],
    "font.size" : 8})

## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": False,
    "font.family": "CMU Serif",
    "font.serif": ["Palatino"],
})


# In[3]:


#expFiles = ["Trimmed_subjectB_folded"+str(i)+".mot" for i in range(1,7)]
#expFiles = ["Trimmed_subjectB_natural"+str(i)+".mot" for i in range(1,7)]
expFiles = ["Trimmed_subjectB_sides"+str(i)+".mot" for i in range(1,7)]

jointNames = ("hip_flexion_r", "knee_angle_r", "ankle_angle_r")

motionFileHeaderLines = 10
startThreshold = 20
endThreshold = 20


# In[4]:


# Experimental Data

jointsData = {}
for jointName in jointNames:
    jointsData.update({jointName:[]})

for i in range(len(expFiles)):
    motionFileName = "../IK/ikResults/"+expFiles[i]
    startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion_r"), (startThreshold,endThreshold), 
                                                motionFileName, motionFileHeaderLines)
    
    motionFile = file(jointNames, motionFileName, motionFileHeaderLines)
    timeData = motionFile.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-5)
    endInd = np.argmax(timeData>=endTime-1e-5)
    
    startTime = timeData[startInd]
    endTime = timeData[endInd]
    
    for j, joint in enumerate(jointNames):
        jointsData[joint].append((startTime, endTime, timeData, motionFile.getColumn(joint)))


# In[5]:


# Simulation Data
yDataNames = ["hip_flexion", "knee_angle", "ankle_angle"]
fileName = "../ProcessedData/00pct/00pct.sto"

startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)),
                                            fileName, 8)

simMotionFile = file(yDataNames, fileName, 8)
simTimeData = simMotionFile.getColumn("time")
startInd = np.argmax(simTimeData>=startTime-1e-5)
endInd = np.argmax(simTimeData>=endTime-1e-5)

simStartTime = simTimeData[startInd]
simEndTime = simTimeData[endInd]
print(simStartTime, simEndTime)


# In[9]:


fig, ax = plt.subplots(7,3, sharey='row', sharex='col',figsize=(7, 10), dpi=300)

for i, joint in enumerate(jointNames):    
    # Individual trial trajectory
    for j in range(len(jointsData[joint])):
        (startTime,endTime, time, data) = jointsData[joint][j]
        ax[j,i].plot(time, data, alpha=0.5)
        ax[j,i].plot((startTime, startTime),(data.min(),data.max()), color="black", alpha=0.5)
        ax[j,i].plot((endTime, endTime),(data.min(),data.max()), color="black", alpha=0.5)
    
    n = np.rad2deg(1)
    simJointData = n*simMotionFile.getColumn(yDataNames[i])
    ax[6,i].plot(simTimeData, simJointData)
    ax[6,i].plot((simStartTime, simStartTime),(simJointData.min(),simJointData.max()), color="black",alpha=0.5)
    ax[6,i].plot((simEndTime, simEndTime),(simJointData.min(),simJointData.max()), color="black",alpha=0.5)
    
fig.set_tight_layout(True)
plt.savefig("figures/Thresholds.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[ ]:




