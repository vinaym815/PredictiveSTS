#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np
import re
from fileReader import file
from fileReader import computePhaseTimes


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


# In[4]:


def plotKinematics(ax, file1, file2, file3, file4):
    n = np.rad2deg(1);
    ax[0].plot(file1.getColumn("time"), n*file1.getColumn("hip_flexion"), "green",label="Hip")
    ax[0].plot(file1.getColumn("time"), n*file1.getColumn("knee_angle"), "darkviolet",label="Knee")
    ax[0].plot(file1.getColumn("time"), n*file1.getColumn("ankle_angle"), "deepskyblue",label="Ankle")
    
    ax[1].plot(file2.getColumn("time"), n*file2.getColumn("hip_flexion"), "green",label="Hip")
    ax[1].plot(file2.getColumn("time"), n*file2.getColumn("knee_angle"), "darkviolet",label="Knee")
    ax[1].plot(file2.getColumn("time"), n*file2.getColumn("ankle_angle"), "deepskyblue",label="Ankle")
    
    (e1, e2, e3) = computePhaseTimes(file4, file1, file3)
    
    limits = [(-20, 120), (-500, 100)]
    for limit, axes in zip(limits, ax):
        axes.plot([e1,e1], limit, linestyle='-.', color="k", alpha=0.5)
        axes.plot([e2,e2], limit, linestyle='-.', color="k", alpha=0.5)
        axes.plot([e3,e3], limit, linestyle='-.', color="k", alpha=0.5)
    
    ax[0].legend(loc="upper right")
    ax[1].legend(loc="lower right")
    
    return None


# In[5]:


models = ["00pct", "20pct", "40pct", "60pctAssisted"]

files = []
for model in models:
    yDataNames = ["ankle_angle", "knee_angle", "hip_flexion"]
    file1 = file(yDataNames, "Data/"+model+".sto", 8)
    
    yDataNames = ["ankle_angle", "knee_angle", "hip_flexion"]
    file2 = file(yDataNames, "Data/"+model+".sto", 8, regrexHeader = r"(\w+)/speed$")
    
    yDataNames = ["com_vel_x"]
    file3 = file(yDataNames, "Data/"+model+"_com.mot", 14)
    
    yDataNames = ["seatConstraint_ground_Fy"]
    file4 = file(yDataNames, "Data/"+model+"_force.mot", 14)
    
    files.append((file1, file2, file3, file4))


# In[6]:


fig, axs = plt.subplots(2, 4, sharex='col', sharey='row', figsize=(11, 5), dpi=300)

plotKinematics(axs[:,0], *files[0])
plotKinematics(axs[:,1], *files[1])
plotKinematics(axs[:,2], *files[2])
plotKinematics(axs[:,3], *files[3])

axs[1,0].set_xlim([0, 1.6])
axs[1,1].set_xlim([0, 1.6])
axs[1,2].set_xlim([0, 1.6])
axs[1,3].set_xlim([0, 1.6])

axs[0,0].set_ylim([-20, 120])
axs[1,0].set_ylim([-500, 100])

fig.text(0.5, -0.01, 'time (s)', ha='center', fontsize=15)
#fig.text(-0.01, 0.5, 'Torques (Nm)', va='center', rotation='vertical', fontsize=15)
fig.text(0.1, 1, '0\% Strength Deficit', fontsize=10)
fig.text(0.34, 1, '20\% Strength Deficit', fontsize=10)
fig.text(0.575, 1, '40\% Strength Deficit', fontsize=10)
fig.text(0.76, 1, '60\% Strength Deficit Externally Assisted', fontsize=10)

fig.set_tight_layout(True)

#plt.savefig("JointKinematics.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[7]:


plt.figure(figsize=(8.15, 2), dpi=300)

yDataNames = ["ankle_angle", "knee_angle", "hip_flexion"]
motion_file = file(yDataNames, "Data/00pct.sto", 8)

yDataNames = ["com_vel_x"]
com_file = file(yDataNames, "Data/00pct_com.mot", 14) 

yDataNames = ["seatConstraint_ground_Fy"]
force_file = file(yDataNames, "Data/00pct_force.mot", 14)

ylim = [-5,140]

time = motion_file.getColumn("time")
(e1, e2, e3) = computePhaseTimes(force_file, motion_file, com_file)

plt.plot(time, np.rad2deg(1)*motion_file.getColumn("hip_flexion"), label="Hip")
plt.plot(time, np.rad2deg(1)*motion_file.getColumn("knee_angle"), label="Knee")
plt.plot(time, np.rad2deg(1)*motion_file.getColumn("ankle_angle"), label="Ankle")
plt.plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e3,e3], ylim, linestyle='-.', color="k", alpha=0.5)

plt.xlabel("time (sec)")
plt.ylabel("Angle (Deg)")
plt.xlim([0.0, 1.6])
plt.ylim(ylim)
plt.legend(prop={'size': 8})
plt.savefig("JointAngles_00pct.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[3]:


yDataNames = ["assistFx", "assistFy", "seatConstraint_ground_Fy"]
file1 = file(yDataNames, "Data/60pctAssisted_force.mot", 14)

yDataNames = ["hip_flexion"]
file2 = file(yDataNames, "Data/60pctAssisted.sto", 8)

yDataNames = ["com_vel_x"]
file3 = file(yDataNames, "Data/60pctAssisted_com.mot", 14)

(e1, e2, e3) = computePhaseTimes(file1, file2, file3)

plt.figure(figsize=(8.15, 2), dpi=300)
plt.plot(file1.getColumn("time"), file1.getColumn("assistFx"), label="Horizontal Assistance")
plt.plot(file1.getColumn("time"), file1.getColumn("assistFy"), label="Vertical Assistance")
plt.plot([e1,e1], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
plt.plot([e3,e3], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)

plt.xlabel("time (sec)")
plt.ylabel("Force (N)")
plt.xlim([0.0, 1.6])
plt.ylim([-5, 400.0])
plt.legend(loc="upper right",prop={'size': 8})
plt.savefig("AssistanceForces_60pctAssist.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[4]:


maxAssistX = np.max(file1.getColumn("assistFx"))
maxAssisty = np.max(file1.getColumn("assistFy"))
print("maxAssistFx", "maxAssistFy")
print(maxAssistX, maxAssisty)


# In[ ]:




