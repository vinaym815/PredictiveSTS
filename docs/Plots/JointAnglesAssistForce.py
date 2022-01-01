#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file
from fileReader import computePhaseTimes
from fileReader import computeMotionTimeRange


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
    n = np.rad2deg(1);
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


# In[3]:

modelWeight = 37.5*9.81

yDataNames = ["hip_flexion"]
motionFile = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted.sto", 8)

yDataNames = ["assistFx", "assistFy", "seatConstraint_ground_Fy"]
forceFile = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted_force.mot", 14)

timeData = forceFile.getColumn("time")

(e1, e2) = computePhaseTimes(forceFile, motionFile)

plt.figure(figsize=(8.15, 2), dpi=300)
plt.plot(timeData, (100.0/modelWeight)*forceFile.getColumn("assistFx"), label="Horizontal Assistance")
plt.plot(timeData, (100.0/modelWeight)*forceFile.getColumn("assistFy"), label="Vertical Assistance")

plt.plot([e1,e1], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)

plt.xlabel(r"$t(sec)$")
plt.ylabel(r"$F(\%mg)$")
plt.xlim([0, 1.112])
plt.ylim([-5, 100.0])
plt.legend(loc="upper right",prop={'size': 8})
#plt.savefig("figures/AssistanceForces_80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()

maxAssistX = np.max(forceFile.getColumn("assistFx"))
maxAssisty = np.max(forceFile.getColumn("assistFy"))
print("maxAssistFx", "maxAssistFy")
print(maxAssistX, maxAssisty)


# In[4]:

simFileName = "../ProcessedData/80pctAssisted/80pctAssisted.sto"
yDataNames = ["hip_flexion"]
motionFile = file(yDataNames, simFileName, 8)

yDataNames = ["assistFx", "assistFy", "seatConstraint_ground_Fy"]
forceFile = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted_force.mot", 14)

(e1, e2) = computePhaseTimes(forceFile, motionFile)
colors = plt.cm.tab10(np.linspace(0, 1, 8))
simMuscleDict = {"soleus" :("SOL", "pink"),
              "gastroc":("GAS",colors[1]),
              "tibant":("TA", colors[2]),
              "recfem":("RF", colors[3]),
              "iliopsoas":("ILPSO",colors[4]),
              "hamstrings":("HAMS",colors[5]),
              "glut_max":("GMAX",colors[6]),
              "vasti":("VAS",colors[7])}

regrexHeader = r"(\w+)/activation$"

yDataNames = [muscle for muscle in simMuscleDict.keys()]
simFile = file(yDataNames, simFileName, 8, regrexHeader=regrexHeader)
timeData = simFile.getColumn("time")

fig = plt.figure(figsize=(8.15, 2.35), dpi=300)
ax = fig.add_subplot(1, 1, 1)

for key, value in simMuscleDict.items():
    ax.plot(timeData, simFile.getColumn(key), label=value[0], color=value[1])
   
ax.plot([e1,e1], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
ax.plot([e2,e2], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)

ax.set_xlabel(r"$t(sec)$")
ax.set_ylabel(r"$a$")
ax.set_xlim([0, 1.112])
ax.set_ylim([-0.05, 1.1])

handles, labels = ax.get_legend_handles_labels()
ax.legend(handles[::-1], labels[::-1], fontsize = 6)

#plt.savefig("figures/Activation_80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()



