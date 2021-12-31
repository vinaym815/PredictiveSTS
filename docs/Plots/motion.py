#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file,computeMotionTimeRange


# In[2]:


plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": True,
    "font.sans-serif": ["CMU Sans Serif"],
    "font.size" : 8})

## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "CMU Serif",
    "font.serif": ["Palatino"],
})


# In[3]:


expFiles = []
for i in range(1,7):
    expFiles.append("Trimmed_subjectB_folded"+str(i)+".mot")
    expFiles.append("Trimmed_subjectB_natural"+str(i)+".mot")
    expFiles.append("Trimmed_subjectB_sides"+str(i)+".mot")

jointNames = ["hip_flexion_r", "knee_angle_r", "ankle_angle_r", "lumbar_extension"]

motionFileHeaderLines = 10
nPoints = 100

startThreshold = 20
endThreshold = 20


# In[4]:


# Experimental Data
motionXAxis = np.linspace(0,100, nPoints)

jointsData = {}
for jointName in jointNames:
    jointsData.update({jointName:np.zeros((nPoints, len(expFiles)))})

times=[]
for i in range(len(expFiles)):
    motionFileName = "../IK/ikResults/"+expFiles[i]
    startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion_r"), 
                                                (startThreshold,endThreshold), 
                                                motionFileName, motionFileHeaderLines)
    
    motionFile = file(jointNames, motionFileName, motionFileHeaderLines)
    timeData = motionFile.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-5)
    endInd = np.argmax(timeData>=endTime-1e-5)
    fileXAxis = np.linspace(0,100, endInd-startInd)
    times.append(timeData[endInd]-timeData[startInd])

    
    for j, joint in enumerate(jointNames):
        jointsData[joint][:,i] = np.interp(motionXAxis, fileXAxis, motionFile.getColumn(joint)[startInd:endInd])

        
##patching the hipJointData
#lumbarKey = jointNames.pop()
#jointsData["hip_flexion_r"] -= jointsData[lumbarKey]
#del jointsData[lumbarKey]


# In[5]:


# Simulation Data
yDataNames = ["hip_flexion", "knee_angle", "ankle_angle", "lumbar_extension"]
fileName = "../ProcessedData/00pct/00pct.sto"
n = np.rad2deg(1)

startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                                            fileName, 8)

simMotionFile = file(yDataNames, fileName, 8)
timeData = simMotionFile.getColumn("time")
startInd = np.argmax(timeData>=startTime-1e-3)
endInd = np.argmax(timeData>=endTime-1e-3)

tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)
simData = {}

for i, jointName in enumerate(yDataNames):
    simData[jointNames[i]] = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        n*simMotionFile.getColumn(jointName)[startInd:endInd])

simXAxis = np.linspace(0,100, nPoints)


# In[7]:


fig, ax = plt.subplots(4,1, sharex='col',figsize=(6, 8), dpi=150)
axLabels = (r"$\theta_{hip}(deg)$", r"$\theta_{knee}(deg)$", r"$\theta_{ankle}(deg)$", r"$\theta_{lumbar}(deg)$")
for i, joint in enumerate(jointNames):
    jointDataMat = jointsData[joint]
    mean = np.mean(jointDataMat, axis=1)
    std = np.std(jointDataMat, axis=1)
    
    ## Individual trial trajectory
    for j in range(jointDataMat.shape[1]):
        ax[i].plot(motionXAxis, jointDataMat[:,j], alpha=0.5)
    
    ax[i].plot(motionXAxis, mean, label="Experiment Mean", color='blue', linestyle='dashed', linewidth=1)
    ax[i].fill_between(motionXAxis, mean-2*std, mean+2*std, color = 'blue', alpha=0.1, label="Experiment 2 S.D.")
    #ax[i].plot(simXAxis, simData[joint], color="red", label="Predicted", linewidth=1)
    
    ax[i].set_ylabel(axLabels[i])
    ax[i].legend(loc='upper right')

ax[2].set_xlim([0,100])
ax[2].set_xlabel("$\%STS$")
fig.set_tight_layout(True)
plt.savefig("figures/JointAnglesTotal.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[ ]:




