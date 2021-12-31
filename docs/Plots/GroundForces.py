#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file, computeMotionTimeRange

import scipy as sp
import scipy.fftpack
from scipy import signal


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


nPoints = 100
startThreshold = 20
endThreshold = 20

subjectWeight = 71*9.81
modelWeight = 37.5*9.81


# In[4]:


forceLabels = ("feet_vy", "chair_vy")

forceFileHeaderLines = 6
forceFilePath = "../IK/trials/"

motionFileHeaderLines = 10
motionFilePath = "../IK/ikResults/"

trialFileNames = []
for i in range(1,7):
    forceFileName = forceFilePath+"Trimmed_subjectB_natural"+str(i)+"_GRF.mot"
    motionFileName = motionFilePath+"Trimmed_subjectB_natural"+str(i)+".mot"
    trialFileNames.append((forceFileName, motionFileName))
    
    forceFileName = forceFilePath+"Trimmed_subjectB_folded"+str(i)+"_GRF.mot"
    motionFileName = motionFilePath+"Trimmed_subjectB_folded"+str(i)+".mot"
    trialFileNames.append((forceFileName, motionFileName))
    
    forceFileName = forceFilePath+"Trimmed_subjectB_sides"+str(i)+"_GRF.mot"
    motionFileName = motionFilePath+"Trimmed_subjectB_sides"+str(i)+".mot"
    trialFileNames.append((forceFileName, motionFileName))
    
    


# In[5]:


forceData = {}
for forceName in forceLabels:
    forceData.update({forceName:np.zeros((nPoints, len(trialFileNames)))})

motionXAxis = np.linspace(0,100, nPoints)

for i in range(len(trialFileNames)):
    forceFileName = trialFileNames[i][0]
    motionFileName = trialFileNames[i][1]
    startTime, endTime = computeMotionTimeRange(("time", "hip_flexion_r"), 
                                                (startThreshold,endThreshold), 
                                                motionFileName, motionFileHeaderLines)
    
    forceFile = file(forceLabels, forceFileName, forceFileHeaderLines)
    time = forceFile.getColumn("time")
    startInd = np.argmax(time>=startTime-1e-5)
    endInd = np.argmax(time>=endTime-1e-5)
    forceXAxis = np.linspace(0,100, endInd-startInd)
    
    for force, forceMat in forceData.items():
        forceArray = np.interp(motionXAxis, forceXAxis, forceFile.getColumn(force)[startInd:endInd])
        forceMat[:,i] = 100*np.divide(forceArray,subjectWeight)


# In[6]:


simForceData = {}

forceFileDict = {"feet_vy": ("Fy", "../ProcessedData/00pct/00pct_feetForces.mot", 4, ","),
            "chair_vy": ("seatConstraint_femur_r_Fy", "../ProcessedData/00pct/00pct_force.mot",14,"\t")}

simMotionFileName = "../ProcessedData/00pct/00pct.sto"
startTime ,endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                                            simMotionFileName, 8)

for force, info in forceFileDict.items():
    forceFile = file([info[0]], info[1], info[2],  delimiter=info[3])
    timeData = forceFile.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-5)
    endInd = np.argmax(timeData>=endTime-1e-3)

    timeEquallySpaced = np.linspace(timeData[startInd], timeData[endInd], nPoints)
    forceArray = np.interp(timeEquallySpaced,timeData[startInd:endInd],
                        forceFile.getColumn(info[0])[startInd:endInd])
    forceArrayNormalized = 100*np.divide(forceArray,modelWeight)
    simForceData.update({force:forceArrayNormalized})


# In[8]:


axLabels = (r"FeetForce/BodyWeight", r"ChairForce/BodyWeight")
fig, ax = plt.subplots(2,1, sharex='col',figsize=(4.5, 6), dpi=150)

for i, force in enumerate(forceLabels):
    forceMat = forceData[force]
    mean = np.mean(forceMat, axis=1)
    std = np.std(forceMat, axis=1)
    
    for j in range(forceMat.shape[1]):
        ax[i].plot(motionXAxis, forceMat[:,j], alpha =0.5)

    ax[i].plot(motionXAxis, mean, label= "Experiment Mean", color='blue', linestyle='dashed', linewidth=2.5)
    ax[i].fill_between(motionXAxis, mean-2*std, mean+2*std, color = 'blue', label="Experiment 2SD", alpha=0.1)
    #ax[i].plot(motionXAxis, -simForceData[force], color="red",label="Prediction")
    
    ax[i].set_ylabel(axLabels[i])
    ax[i].legend(loc="upper right")
    #ax[i].set_ylim([-0.05,1.05])
    ax[i].set_xlim([0, 100])
    
ax[1].set_xlabel(r"\%STS")
fig.set_tight_layout(True)
plt.savefig("figures/GroundForcesTotal.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[ ]:




