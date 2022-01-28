#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
from fileReader import file
from fileReader import computePhaseTimes, computeMotionTimeRange


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

startThreshold = np.deg2rad(20)
endThreshold = np.deg2rad(20)
simStartEndnPointMovingAvg = 1


# In[4]:


def plotKinematics(ax, file1, file2, file3):
    n = np.rad2deg(1)
    motionXAxis = np.linspace(0,100, nPoints)
    
    startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), (startThreshold, endThreshold), file1.fileName, 8, simStartEndnPointMovingAvg)

    timeData = file1.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)
    
    (e1, e2) = computePhaseTimes(file3, file1)
    indE1 = np.argmax(tEquallySpace>=e1-1e-3)
    indE2 = np.argmax(tEquallySpace>=e2-1e-3)
    e1 = motionXAxis[indE1]
    e2 = motionXAxis[indE2]
    
    joints = ["hip_flexion", "knee_angle", "ankle_angle"]
    colors = ["green", "darkviolet", "deepskyblue"]
    labels = ["Hip", "Knee", "Ankle"]
    
    for i, joint in enumerate(joints):
        jointData = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        n*file1.getColumn(joint)[startInd:endInd])
        
        jointVelData = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        n*file2.getColumn(joint)[startInd:endInd])

        ax[0].plot(motionXAxis, jointData, colors[i], label=labels[i])
        ax[1].plot(motionXAxis, jointVelData, colors[i], label=labels[i])
    

    
    limits = [(-20, 120), (-250, 150)]
    for limit, axes in zip(limits, ax):
        axes.plot([e1,e1], limit, linestyle='-.', color="k", alpha=0.5)
        axes.plot([e2,e2], limit, linestyle='-.', color="k", alpha=0.5)
    
    ax[0].legend(loc="upper right")
    ax[1].legend(loc="upper right")
    
    return None

# In[5]:


models = ["00pct", "20pct", "40pct", "60pct"]

files = []
for model in models:
    yDataNames = ["ankle_angle", "knee_angle", "hip_flexion"]
    file1 = file(yDataNames, "../ProcessedData/"+model+"/"+model+".sto", 8)
    
    yDataNames = ["ankle_angle", "knee_angle", "hip_flexion"]
    file2 = file(yDataNames, "../ProcessedData/"+model+"/"+model+".sto", 8, regrexHeader = r"(\w+)/speed$")
    
    yDataNames = ["seatConstraint_ground_Fy"]
    file3 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_force.mot", 14)
    
    files.append((file1, file2, file3))


# In[6]:


fig, axs = plt.subplots(2, 4, sharex='col', sharey='row', figsize=(11, 5), dpi=300)

plotKinematics(axs[:,0], *files[0])
plotKinematics(axs[:,1], *files[1])
plotKinematics(axs[:,2], *files[2])
plotKinematics(axs[:,3], *files[3])

axs[1,0].set_xlim([0, 100])
axs[1,1].set_xlim([0, 100])
axs[1,2].set_xlim([0, 100])
axs[1,3].set_xlim([0, 100])

axs[0,0].set_ylim([-20, 120])
axs[1,0].set_ylim([-250, 150])

axs[0,0].set_ylabel(r"$\theta (^\circ)$")
axs[1,0].set_ylabel(r"$\dot{\theta} (^\circ/sec)$")

fig.text(0.5, -0.01, '\% STS', ha='center', fontsize=15)
#fig.text(-0.01, 0.5, 'Torques (Nm)', va='center', rotation='vertical', fontsize=15)
fig.text(0.1, 1, '0\% Strength Deficit', fontsize=10)
fig.text(0.34, 1, '20\% Strength Deficit', fontsize=10)
fig.text(0.575, 1, '40\% Strength Deficit', fontsize=10)
fig.text(0.82, 1, '60\% Strength Deficit', fontsize=10)

fig.set_tight_layout(True)

plt.savefig("figures/JointKinematics.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()
