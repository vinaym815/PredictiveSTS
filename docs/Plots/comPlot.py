#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
#from matplotlib.colors import ListedColormap
from fileReader import file, computeMotionTimeRange, computePhaseTimes


# In[2]:


plt.style.use('ggplot')
plt.rcParams.update({
    "text.usetex": True,
    "font.sans-serif": ["Helvetica"]})
## for Palatino and other serif fonts use:
plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.serif": ["Palatino"],
})


# In[4]:

nPoints = 100
startThreshold = np.deg2rad(20)
endThreshold = np.deg2rad(20)
simStartEndnPointMovingAvg = 1

motionXAxis = np.linspace(0,100, nPoints)

yDataNames = ["time", "com_x", "com_y", "com_vel_x", "com_vel_y"]
comFile = file(yDataNames, "../ProcessedData/00pct/00pct_com.sto", 4, delimiter=",")
motionFileName = "../ProcessedData/00pct/00pct.sto"

startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), (startThreshold, endThreshold), motionFileName, 8, simStartEndnPointMovingAvg)    

timeData = comFile.getColumn("time")
startInd = np.argmax(timeData>=startTime-1e-3)
endInd = np.argmax(timeData>=endTime-1e-3)
tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)

com_x = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        comFile.getColumn("com_x")[startInd:endInd])
com_y = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        comFile.getColumn("com_y")[startInd:endInd])        
com_vel_x = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        comFile.getColumn("com_vel_x")[startInd:endInd])
com_vel_y = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        comFile.getColumn("com_vel_y")[startInd:endInd])


# In[6]:


points = np.array([com_x, com_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, axs = plt.subplots(1, 1, figsize=(5, 5), dpi=300)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(0.0, 100)
lc = LineCollection(segments, cmap='jet', norm=norm)
# Set the values used for colormapping
lc.set_array(motionXAxis)
lc.set_linewidth(2)
line = axs.add_collection(lc)
colorbar = fig.colorbar(line, ax=axs)

axs.axis('equal')
colorbar.set_label(r"$\%STS$")
axs.set_xlabel(r"$COM_x(m)$")
axs.set_ylabel(r"$COM_y(m)$")
#axs.set_ylim(0.65, 0.96)
axs.set_xlim(-0.2, 0.1)

#plt.savefig("figures/comXY_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()


# In[7]:
    
    
jointNames = ["hip_flexion"]
motion_file = file(jointNames, "../ProcessedData/00pct/00pct.sto", 8)
yDataNames = ["seatConstraint_ground_Fy"]
force_file = file(yDataNames, "../ProcessedData/00pct/00pct_force.mot", 14)
    
(e1, e2) = computePhaseTimes(force_file, motion_file)
indE1 = np.argmax(tEquallySpace>=e1-1e-3)
indE2 = np.argmax(tEquallySpace>=e2-1e-3)
e1 = motionXAxis[indE1]
e2 = motionXAxis[indE2]
ylim = [-0.2, 1.0]

plt.figure(figsize=(5, 5), dpi=300)
plt.plot(motionXAxis, com_vel_x, label=r"$\dot{COM_x}$")
plt.plot(motionXAxis, com_vel_y, label=r"$\dot{COM_y}$")
plt.plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)

plt.xlim([0.0, 100])
#plt.ylim([-0.2, 0.8])
plt.xlabel(r"$\%STS$")
plt.ylabel(r"$Velocity (m/s)$")
plt.legend(prop={'size': 8})
#plt.savefig("figures/comVel_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()


# In[8]:


heelX = 0.0
toeX = 0.2088

yDataNames = ["Fy", "Mz"]
feetFileName = "../ProcessedData/00pct/00pct_feetForces.mot"
feetFile = file(yDataNames, feetFileName, 4, ",")

timeData = feetFile.getColumn("time")
startInd = np.argmax(timeData>=startTime-1e-3)
endInd = np.argmax(timeData>=endTime-1e-3)
tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)

fY = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        feetFile.getColumn("Fy")[startInd:endInd])
mZ = np.interp(tEquallySpace, timeData[startInd:endInd], 
                                        feetFile.getColumn("Mz")[startInd:endInd])
zmpVec = np.divide(mZ,fY)
ylim = [-0.20,0.225]
xlim = [0,100]

plt.figure(figsize=(5.5, 3.5), dpi=300)
plt.plot(motionXAxis, com_x, label=r"$COM_x$")
plt.plot(motionXAxis, zmpVec, label=r"$ZMP_x$")
plt.plot(xlim, [heelX, heelX], label=r"$Heel_x$", color="green", alpha=0.85)
plt.plot(xlim, [toeX, toeX], label=r"$Toes_x$", color="darkviolet", alpha=0.85)

plt.plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)

plt.xlim([0.0, 100])
plt.ylim(ylim)
plt.xlabel(r"$\%STS$")
plt.ylabel(r"$Distance (m)$")
plt.legend(prop={'size': 8})
plt.savefig("figures/zmp_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[9]:
    
yDataNames = ["time", "com_vel_y", "com_vel_x"]
models = ["00", "20", "40", "60"]
fileNames = ["../ProcessedData/" + model + "pct/"+ model +"pct_com.sto" for model in models]

for fileName in fileNames:
    fileData = file(yDataNames, fileName, 4, delimiter=",")
    print(np.max(fileData.getColumn("com_vel_x")))
    
    
    
    
