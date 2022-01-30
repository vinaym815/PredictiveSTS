#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file, computeMotionTimeRange, computePhaseTimes


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

expStartEndnPointMovingAvg = 20
simStartEndnPointMovingAvg = 1


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
                                                motionFileName, motionFileHeaderLines, expStartEndnPointMovingAvg)
    
    motionFile = file(jointNames, motionFileName, motionFileHeaderLines)
    timeData = motionFile.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-5)
    endInd = np.argmax(timeData>=endTime-1e-5)
    fileXAxis = np.linspace(0,100, endInd-startInd)
    times.append(timeData[endInd]-timeData[startInd])
    
    for j, joint in enumerate(jointNames):
        jointsData[joint][:,i] = np.interp(motionXAxis, fileXAxis, motionFile.getColumn(joint)[startInd:endInd])


# In[5]:


# Simulation Data
yDataNames = ["hip_flexion", "knee_angle", "ankle_angle", "lumbar_extension"]
fileName = "../ProcessedData/00pct/00pct.sto"
n = np.rad2deg(1)

startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                                            fileName, 8, simStartEndnPointMovingAvg)

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

yDataNames = ["seatConstraint_ground_Fy"]
simForceFile = file(yDataNames, "../ProcessedData/00pct/00pct_force.mot", 14)
(e1, e2) = computePhaseTimes(simForceFile, simMotionFile)
indE1 = np.argmax(tEquallySpace>=e1-1e-3)
indE2 = np.argmax(tEquallySpace>=e2-1e-3)
e1 = motionXAxis[indE1]
e2 = motionXAxis[indE2]
ylim = [-100, 100]

def pctToTime(pctVec, t1, t2):
    val = []
    for pct in pctVec:
        val.append(t1+(t2-t1)*pct/100)
    return val

#vin = [0,20,40,60,80, 100]
#print("vin")
#print(startTime, endTime)
#print(pctToTime(vin, startTime, endTime))


# In[7]:

rows = 2
cols = 2
limits = [[-17.5,110],[-5,90],[-15,20],[-42,10]]

fig, ax = plt.subplots(rows, cols, sharex='col', figsize=(6, 4), dpi=300)
axLabels = (r"$\theta_{hip}$", r"$\theta_{knee}$", r"$\theta_{ankle}$", r"$\theta_{lumbar}$")
for i, joint in enumerate(jointNames):
    jointDataMat = jointsData[joint]
    mean = np.mean(jointDataMat, axis=1)
    std = np.std(jointDataMat, axis=1)
    
    row = i//cols
    col = i%cols

    ## Individual trial trajectory
    #for j in range(jointDataMat.shape[1]):
    #    ax[row,col].plot(motionXAxis, jointDataMat[:,j], alpha=0.8)
    
    ax[row,col].plot(simXAxis, simData[joint], color="red", label="Simulation", linewidth=1)
    ax[row,col].plot(motionXAxis, mean, label="Experiment Mean", color='blue', linestyle='dashed', linewidth=1.5)
    ax[row,col].fill_between(motionXAxis, mean-2*std, mean+2*std, color = 'blue', alpha=0.2, label=r"Experiment Mean $\pm$ 2 S.D.")
    
    ax[row,col].set_title(axLabels[i])
    ax[row,col].plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
    ax[row,col].plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)
    ax[row,col].set_ylim(limits[i])
    #ax[row,col].legend()

#ax[3].set_ylim(0,100)
ax[1,0].set_xlim([0,100])
ax[1,1].set_xlim([0,100])
ax[1,0].set_xlabel("$\%STS$")
ax[0,0].set_ylabel(r"$\theta(^\circ)$")
ax[1,0].set_ylabel(r"$\theta(^\circ)$")
fig.set_tight_layout(True)
plt.savefig("figures/JointAnglesComparison.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()
