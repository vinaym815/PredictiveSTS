# In[1]:


from matplotlib import pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from fileReader import file, computePhaseTimes


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

yDataNames = ["time", "com_x", "com_y", "com_vel_x", "com_vel_y"]
comFile = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted_com.sto", 4, delimiter=",")

timeData = comFile.getColumn("time")

com_x = comFile.getColumn("com_x")
com_y = comFile.getColumn("com_y")        
com_vel_x = comFile.getColumn("com_vel_x")
com_vel_y = comFile.getColumn("com_vel_y")


# In[6]:


points = np.array([com_x, com_y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

fig, axs = plt.subplots(1, 1, figsize=(5, 5), dpi=300)

# Create a continuous norm to map from data points to colors
norm = plt.Normalize(0.0, 100)
lc = LineCollection(segments, cmap='jet', norm=norm)
# Set the values used for colormapping
lc.set_array(timeData)
lc.set_linewidth(2)
line = axs.add_collection(lc)
colorbar = fig.colorbar(line, ax=axs)

axs.axis('equal')
colorbar.set_label(r"$t(s)$")
axs.set_xlabel(r"$COM_x(m)$")
axs.set_ylabel(r"$COM_y(m)$")
#axs.set_ylim(0.65, 0.96)
axs.set_xlim(-0.2, 0.1)

plt.savefig("figures/comXY_80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[7]:
    
    
jointNames = ["hip_flexion"]
motion_file = file(jointNames, "../ProcessedData/80pctAssisted/80pctAssisted.sto", 8)
yDataNames = ["seatConstraint_ground_Fy"]
force_file = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted_force.mot", 14)
    
(e1, e2) = computePhaseTimes(force_file, motion_file)
ylim = [-0.2, 1.0]

plt.figure(figsize=(5, 5), dpi=300)
plt.plot(timeData, com_vel_x, label=r"$\dot{COM_x}$")
plt.plot(timeData, com_vel_y, label=r"$\dot{COM_y}$")
plt.plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)

plt.xlim([0.0, 1.11])
#plt.ylim([-0.2, 0.8])
plt.xlabel(r"$t (s)$")
plt.ylabel(r"$Velocity (m/s)$")
plt.legend(prop={'size': 8})
plt.savefig("figures/comVel_80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[8]:

timeDataCOMFile = timeData
heelX = 0.0
toeX = 0.2088

yDataNames = ["Fy", "Mz"]
feetFileName = "../ProcessedData/80pctAssisted/80pctAssisted_feetForces.mot"
feetFile = file(yDataNames, feetFileName, 4, ",")

timeData = feetFile.getColumn("time")
fY = feetFile.getColumn("Fy")
mZ = feetFile.getColumn("Mz")
zmpVec = np.divide(mZ,fY)
ylim = [-0.20,0.225]
xlim = [0,100]

plt.figure(figsize=(5.5, 3.5), dpi=300)
plt.plot(timeDataCOMFile, com_x, label=r"$COM_x$")
plt.plot(timeData, zmpVec, label=r"$ZMP_x$")
plt.plot(xlim, [heelX, heelX], label=r"$Heel_x$", color="green", alpha=0.85)
plt.plot(xlim, [toeX, toeX], label=r"$Toes_x$", color="darkviolet", alpha=0.85)

plt.plot([e1,e1], ylim, linestyle='-.', color="k", alpha=0.5)
plt.plot([e2,e2], ylim, linestyle='-.', color="k", alpha=0.5)

plt.xlim([0.0, 1.11])
plt.ylim(ylim)
plt.xlabel(r"$t (s)$")
plt.ylabel(r"$Distance (m)$")
plt.legend(prop={'size': 8})
plt.savefig("figures/zmp_80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()

print(np.max(com_vel_x), np.max(com_vel_y))


# %%

simFileName = "../ProcessedData/80pctAssisted/80pctAssisted.sto"
modelWeight = 37.5*9.81
regrexHeader = r"(\w+)/activation$"
colors = plt.cm.tab10(np.linspace(0, 1, 8))
simMuscleDict = {"soleus" :("SOL", "pink"),
              "gastroc":("GAS",colors[1]),
              "tibant":("TA", colors[2]),
              "recfem":("RF", colors[3]),
              "iliopsoas":("ILPSO",colors[4]),
              "hamstrings":("HAMS",colors[5]),
              "glut_max":("GMAX",colors[6]),
              "vasti":("VAS",colors[7])}

yDataNames = [muscle for muscle in simMuscleDict.keys()]
simFile = file(yDataNames, simFileName, 8, regrexHeader=regrexHeader)

yDataNames = ["hip_flexion"]
motionFile = file(yDataNames, simFileName, 8)

yDataNames = ["assistFx", "assistFy", "seatConstraint_ground_Fy"]
forceFile = file(yDataNames, "../ProcessedData/80pctAssisted/80pctAssisted_force.mot", 14)

(e1, e2) = computePhaseTimes(forceFile, motionFile)

#%%
fig, axs = plt.subplots(2, 1, sharex='col', figsize=(6.5, 4.5), dpi=300)

axs[0].plot(forceFile.getColumn("time"), (100.0/modelWeight)*forceFile.getColumn("assistFx"), label="Horizontal Assistance")
axs[0].plot(forceFile.getColumn("time"), (100.0/modelWeight)*forceFile.getColumn("assistFy"), label="Vertical Assistance")

for key, value in simMuscleDict.items():
    axs[1].plot(simFile.getColumn("time"), simFile.getColumn(key), label=value[0], color=value[1])

axs[0].plot([e1,e1], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
axs[0].plot([e2,e2], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
axs[1].plot([e1,e1], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)
axs[1].plot([e2,e2], [-5, 400.0], linestyle='-.', color="k", alpha=0.5)

axs[0].set_ylabel(r"$F(\%mg)$")
axs[1].set_ylabel(r"$a$")
axs[1].set_xlabel(r"$t(sec)$")

axs[1].set_xlim([0, 1.112])
axs[1].set_ylim([-0.05, 1.2])
axs[0].set_ylim([-5, 100.0])

axs[0].legend(loc="upper right",prop={'size': 8})
handles, labels = axs[1].get_legend_handles_labels()
axs[1].legend(handles[::-1], labels[::-1], ncol=2, fontsize=7)

fig.set_tight_layout(True)
plt.savefig("figures/80pctAssisted.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# %% 
maxAssistX = np.max(forceFile.getColumn("assistFx"))
maxAssisty = np.max(forceFile.getColumn("assistFy"))
print("maxAssistFx", "maxAssistFy")
print(maxAssistX, maxAssisty)
