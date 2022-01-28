#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
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


nPoints = 100
startThreshold = 20
endThreshold = 20
expStartEndnPointMovingAvg = 20
simStartEndnPointMovingAvg = 1

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
                                                motionFileName, motionFileHeaderLines, expStartEndnPointMovingAvg)
    
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
                                            simMotionFileName, 8, simStartEndnPointMovingAvg)

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

yDataNames = ["hip_flexion"]
simMotionFile = file(yDataNames, "../ProcessedData/00pct/00pct.sto", 8)
yDataNames = ["seatConstraint_ground_Fy"]
simForceFile = file(yDataNames, "../ProcessedData/00pct/00pct_force.mot", 14)
(e1, e2) = computePhaseTimes(simForceFile, simMotionFile)
indE1 = np.argmax(timeEquallySpaced>=e1-1e-3)
indE2 = np.argmax(timeEquallySpaced>=e2-1e-3)
e1 = motionXAxis[indE1]
e2 = motionXAxis[indE2]
ylim = [-15, 175]


# In[8]:


axLabels = (r"$F_{feet,y}(\%mg)$", r"$F_{chair,y}(\%mg)$")
fig, ax = plt.subplots(1,2, figsize=(6, 2.4), dpi=300)

for i, force in enumerate(forceLabels):
    forceMat = forceData[force]
    mean = np.mean(forceMat, axis=1)
    std = np.std(forceMat, axis=1)
    
    for j in range(forceMat.shape[1]):
        ax[i].plot(motionXAxis, forceMat[:,j], alpha=0.8)
        
    ax[i].plot(motionXAxis, mean, label= r"$Experiment Mean$", color='blue', linestyle='dashed', linewidth=2.5)
    ax[i].fill_between(motionXAxis, mean-2*std, mean+2*std, color = 'blue', label=r"$Experiment Mean \pm 2SD$", alpha=0.2) 
    #ax[i].plot(motionXAxis, -simForceData[force], color="red",label="Prediction")
    
    #ax[i].plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
    #ax[i].plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)

    ax[i].set_ylabel(axLabels[i])
    ax[i].set_xlim([0, 100])    
    ax[i].set_ylim([-10, 160])    

ax[0].set_xlabel(r"\%STS")
fig.set_tight_layout(True)
plt.savefig("figures/GroundForcesExperimentF.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[ ]:




