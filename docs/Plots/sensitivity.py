# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import os
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

colors = plt.cm.tab10(np.linspace(0, 1, 11))
labels = [r"$\phi_1$", r"$\phi_9$", r"$\phi_{10}$", r"$\phi_2$", r"$\phi_3$", 
            r"$\phi_4$", r"$\phi_6$", r"$\phi_8$", r"$\phi_7$"]
titles = [r"$\theta_{hip}$", r"$\theta_{knee}$", r"$\theta_{ankle}$", r"$\theta_{lumbar}$"]
limits = [[-17.5,110],[-10,95],[-15,20],[-42,10]]

path = "../ProcessedData/Sensitivity/"
dirContents = os.listdir(path)
fileNames = [path+directory+"/optimTraj.sto" for directory in dirContents if os.path.isdir(path+directory)]

jointNames = ("hip_flexion", "knee_angle", "ankle_angle", "lumbar_extension")
musclesDict = {"GMAX" :"glut_max", 
                   "HAMS":"hamstrings",
                   "VAS":"vasti", 
                   "RF":"recfem", 
                   "TA":"tibant",
                   "GAS":"gastroc",
                   "SOL":"soleus",
                   "ILPSO":"iliopsoas"}

regrexHeader = r"(\w+)/activation$"

nHeaderLines = 8
nPoints = 100

n = np.rad2deg(1)
startThreshold = 20
endThreshold = 20
simStartEndnPointMovingAvg = 1

# In[4]:

motionXAxis = np.linspace(0,100, nPoints)

newData = {}
for jointName in jointNames:
    newData.update({jointName:np.zeros((nPoints, len(fileNames)))})

for muscleName in musclesDict.keys():
    newData.update({muscleName:np.zeros((nPoints, len(fileNames)))})

for i, fileName in enumerate(fileNames):
    startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                                            (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                                            fileName, nHeaderLines, simStartEndnPointMovingAvg)

    motionFile = file(jointNames, fileName, nHeaderLines)
    emgFile = file(musclesDict.values(), fileName, nHeaderLines, regrexHeader=regrexHeader)

    timeData = motionFile.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)

    tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)

    for jointName in jointNames:
        newData[jointName][:, i] = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                    n*motionFile.getColumn(jointName)[startInd:endInd])

    for key, muscleName in musclesDict.items():
        newData[key][:, i] = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                emgFile.getColumn(muscleName)[startInd:endInd])

# %%

baseData = {}
fileName = "../ProcessedData/00pct/00pct.sto"

startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), 
                        (np.deg2rad(startThreshold), np.deg2rad(endThreshold)), 
                        fileName, nHeaderLines, simStartEndnPointMovingAvg)

motionFile = file(jointNames, fileName, nHeaderLines)
emgFile = file(musclesDict.values(), fileName, nHeaderLines, regrexHeader=regrexHeader)
timeData = motionFile.getColumn("time")
startInd = np.argmax(timeData>=startTime-1e-3)
endInd = np.argmax(timeData>=endTime-1e-3)

tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)

for jointName in jointNames:
    baseData.update({jointName : np.interp(tEquallySpace, timeData[startInd:endInd], 
                    n*motionFile.getColumn(jointName)[startInd:endInd])})
for key, value in musclesDict.items():
    baseData.update({key:np.interp(tEquallySpace, timeData[startInd:endInd], 
                            emgFile.getColumn(value)[startInd:endInd])})

# In[5]:

rows = 3
cols = 4

fig, axs = plt.subplots(rows, cols, sharex='col', figsize=(8.2, 6), dpi=300)

for i, jointName in enumerate(jointNames):
    row = i//cols
    col = i%cols

    for j in range(len(fileNames)):
        axs[row,col].plot(motionXAxis, newData[jointName][:,j], color=colors[j], alpha=0.85, label=labels[j])
    
    axs[row,col].plot(motionXAxis, baseData[jointName], 'k--', label="Base", linewidth=2)

    axs[row,col].set_ylim(limits[i]) 
    axs[row,col].set_title(titles[i])
    #axs[row,col].legend()

for i, key in enumerate(musclesDict.keys()):
    i += 4
    row = i//cols
    col = i%cols

    for j in range(len(fileNames)):
        axs[row,col].plot(motionXAxis, newData[key][:,j], color=colors[j], alpha=0.85, label=labels[j])
    
    axs[row,col].plot(motionXAxis, baseData[key], "k--", linewidth=2)
    
    axs[row,col].set_title(key)
    axs[row,col].set_xlim([0, 100])
    axs[row,col].set_ylim([-0.05, 1.05])
    if(col==0): axs[row,col].set_ylabel(r"$a$")
    #axs[row,col].legend(loc="upper left")

axs[0,0].set_ylabel(r"$\theta (^\circ)$")
axs[1,0].set_ylabel(r"$a$")
axs[2,0].set_ylabel(r"$a$")
axs[2,0].set_xlabel(r"$\% STS$")



fig.set_tight_layout(True)
plt.savefig("figures/Sensitivity.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()

# %%
