#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np 
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


# In[3]:


def plotActuation(ax, file1, file2, file3, file4, file5, file6, file7, file8):
    colors = plt.cm.tab10(np.linspace(0, 1, 8))
    
    for file_, axes in zip((file1, file2), ax):
        axes.plot(file_.getColumn("time"), file_.getColumn("soleus"), label="SOL", color="pink")
        axes.plot(file_.getColumn("time"), file_.getColumn("gastroc"), label="GAS", color=colors[1])
        axes.plot(file_.getColumn("time"), file_.getColumn("tibant"), label="TA", color=colors[2])
        axes.plot(file_.getColumn("time"), file_.getColumn("recfem"), label="RF", color=colors[3])
        axes.plot(file_.getColumn("time"), file_.getColumn("iliopsoas"), label="ILPSO", color=colors[4])
        axes.plot(file_.getColumn("time"), file_.getColumn("hamstrings"), label="HAMS", color=colors[5])
        axes.plot(file_.getColumn("time"), file_.getColumn("glut_max"), label="GMAX", color=colors[6])
        axes.plot(file_.getColumn("time"), file_.getColumn("vasti"), label="VAS", color=colors[7])        
    
    ax[2].plot(file4.getColumn("time"), file4.getColumn("iliopsoas"), label="ILPSO", color=colors[4])
    ax[2].plot(file4.getColumn("time"), file4.getColumn("recfem"), label="RF", color=colors[3])
    ax[2].plot(file4.getColumn("time"), file4.getColumn("hamstrings"), label="HAMS", color=colors[5])
    ax[2].plot(file4.getColumn("time"), file4.getColumn("glut_max"), label="GMAX", color=colors[6])
    ax[2].plot(file3.getColumn("time"), file3.getColumn("hip_flexion_moment"), "--",color="green",linewidth=2, label="Hip")

    ax[3].plot(file5.getColumn("time"), file5.getColumn("gastroc"), label="GAS", color=colors[1])
    ax[3].plot(file5.getColumn("time"), file5.getColumn("recfem"), label="RF", color=colors[3])
    ax[3].plot(file5.getColumn("time"), file5.getColumn("hamstrings"), label="HAMS", color=colors[5])
    ax[3].plot(file5.getColumn("time"), file5.getColumn("vasti"), label="VAS", color=colors[7])
    ax[3].plot(file3.getColumn("time"), file3.getColumn("knee_angle_moment"), "--",color="darkviolet",linewidth=2, label="Knee")
    
    ax[4].plot(file6.getColumn("time"), file6.getColumn("gastroc"), label="GAS", color=colors[1])
    ax[4].plot(file6.getColumn("time"), file6.getColumn("soleus"), label="SOL", color="pink")
    ax[4].plot(file6.getColumn("time"), file6.getColumn("tibant"), label="TA", color=colors[2])
    ax[4].plot(file3.getColumn("time"), file3.getColumn("ankle_angle_moment"), "--",color="deepskyblue",linewidth=2, label="Ankle")
    
    (e1, e2, e3) = computePhaseTimes(file2, file7, file8)
    limits = [(-0.05, 1.05), (0.0, 16000), (-175, 120), (-320, 110), (-151, 150)]
    
    for axes, limit, i in zip(ax, limits, range(len(limits))):
        axes.plot([e1,e1], limit, linestyle='-.', color="k", alpha=0.3)
        axes.plot([e2,e2], limit, linestyle='-.', color="k", alpha=0.3)
        axes.plot([e3,e3], limit, linestyle='-.', color="k", alpha=0.3)
        
        handlelength=2.5
        loc = "lower right"        
        if i==0:
            handlelength=1
        if (i== 0 or i ==1):
            loc = "upper right"
            
        handles, labels = axes.get_legend_handles_labels()
        axes.legend(handles[::-1], labels[::-1], loc=loc, fontsize = 6, handlelength=handlelength)

    return None


# In[4]:


models = ["00pct", "20pct", "40pct", "60pctAssisted"]

files = []
for model in models:
    
    yDataNames = ["gastroc", "glut_max", "iliopsoas", "recfem", "tibant", "hamstrings", "soleus","tibant", "vasti"]
    regrexHeader = r"(\w+)/activation$"
    file1 = file(yDataNames, "Data/"+model+".sto", 8, regrexHeader=regrexHeader)
                 
    yDataNames = ["gastroc", "glut_max", "iliopsoas", "recfem", "tibant", "hamstrings", "soleus","tibant", "vasti", 
                  "seatConstraint_ground_Fy"]
    file2 = file(yDataNames, "Data/"+model+"_force.mot", 14)
    
    yDataNames = ["ankle_angle_moment", "knee_angle_moment", "hip_flexion_moment"]
    file3 = file(yDataNames, "Data/"+model+"_ID.sto", 6)
    
    yDataNames = ["recfem", "hamstrings", "glut_max", "iliopsoas"]
    file4 = file(yDataNames, "Data/"+model+"_ID_Hip.sto", 11)
    
    yDataNames = ["gastroc", "recfem", "hamstrings", "vasti"]
    file5 = file(yDataNames, "Data/"+model+"_ID_Knee.sto", 11)
    
    yDataNames = ["gastroc", "soleus", "tibant"]
    file6 = file(yDataNames, "Data/"+model+"_ID_Ankle.sto", 11)
        
    yDataNames = ["hip_flexion"]
    file7 = file(yDataNames, "Data/"+model+".sto", 8)
    
    yDataNames = ["com_vel_x", "com_vel_y"]
    file8 = file(yDataNames, "Data/"+model+"_com.mot", 14)
    
    files.append((file1, file2, file3, file4, file5 ,file6, file7, file8))


# In[5]:


fig, axs = plt.subplots(5, 4, sharex='col', sharey='row', figsize=(8.2, 10.0), dpi=300)

plotActuation(axs[:,0], *files[0])
plotActuation(axs[:,1], *files[1])
plotActuation(axs[:,2], *files[2])
plotActuation(axs[:,3], *files[3])

limits = [(-0.05, 1.05), (0.0, 16000), (-175, 120), (-320, 110), (-151, 150)]

for i in range(4):
    axs[4,i].set_xlim([0, 1.6])

for i,limit in enumerate(limits):
    axs[i,0].set_ylim(limit)

fig.text(0.5, -0.01, 'time (s)', ha='center', fontsize=10)
fig.text(-0.005, 0.12, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.31, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.5, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.72, 'Force (N)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.9, 'Activation', va='center', rotation='vertical', fontsize=10)

fig.text(0.1, 1, '0\% Strength Deficit', fontsize=10)
fig.text(0.32, 1, '20\% Strength Deficit', fontsize=10)
fig.text(0.55, 1, '40\% Strength Deficit', fontsize=10)
fig.text(0.75, 1, '60\% Strength Deficit (Assisted)', fontsize=10)

fig.set_tight_layout(True)

plt.savefig("Actuation.png", format="png",transparent=False, bbox_inches = 'tight')
plt.show()


# In[16]:


def peakTorqueKnee(file1, file2):
    ind = np.argmin(file1.getColumn("knee_angle_moment"))
    t = file1.getColumn("time")[ind]
    peakTorque = file1.getColumn("knee_angle_moment")[ind]
    
    ind = np.argmax(file2.getColumn("time")>= t)
    vasTorque = file2.getColumn("vasti")[ind]
    hamsTorque = file2.getColumn("hamstrings")[ind]
    rfTorque = file2.getColumn("recfem")[ind]
    gasTorque = file2.getColumn("gastroc")[ind]
    
    return (peakTorque, vasTorque, hamsTorque, rfTorque, gasTorque,t)

def peakTorqueHip(file1, file2):
    ind = np.argmin(file1.getColumn("hip_flexion_moment"))
    t = file1.getColumn("time")[ind]
    peakTorque = file1.getColumn("hip_flexion_moment")[ind]
    
    ind = np.argmax(file2.getColumn("time")>= t)
    gmaxTorque = file2.getColumn("glut_max")[ind]
    hamsTorque = file2.getColumn("hamstrings")[ind]
    rfTorque = file2.getColumn("recfem")[ind]
    iliopTorque = file2.getColumn("iliopsoas")[ind]
    
    return (peakTorque, gmaxTorque, hamsTorque, rfTorque, iliopTorque,t)


# In[17]:


for vin in files:
    print(peakTorqueKnee(vin[2], vin[4]))


# In[18]:


for vin in files:
    print(peakTorqueHip(vin[2], vin[3]))


# In[19]:


# time difference peak hip and knee torque
for vin in files:
    peakKnee = peakTorqueKnee(vin[2], vin[4])
    peakHip = peakTorqueHip(vin[2], vin[3])
    print(peakKnee[-1] - peakHip[-1])


# In[9]:


# Maximum contribution to knee torque
for vin in files:
    peakVasKnee = np.min(vin[4].getColumn("vasti"))
    peakHamsKnee = np.max(vin[4].getColumn("hamstrings"))
    peakRfKnee = np.min(vin[4].getColumn("recfem"))
    peakGasKnee = np.min(vin[4].getColumn("gastroc"))
    print((peakVasKnee, peakHamsKnee,peakRfKnee, peakGasKnee))


# In[10]:


# Maximum contribution to hip torque
for vin in files:
    peakGMaxHip = np.min(vin[3].getColumn("glut_max"))
    peakHamsHip = np.min(vin[3].getColumn("hamstrings"))
    peakRfHip = np.min(vin[3].getColumn("recfem"))
    peakIlipsoHip = np.min(vin[3].getColumn("iliopsoas"))
    print((peakGMaxHip, peakHamsHip, peakRfHip, peakIlipsoHip))


# In[11]:


for vin in files:
    print(np.min(vin[2].getColumn("ankle_angle_moment")))


# In[12]:


for vin in files:
    (e1, e2, e3) = computePhaseTimes(vin[1], vin[6], vin[7])
    print(e3)


# In[13]:


# Maximum muscle forces
for vin in files:
    maxVas = np.max(vin[1].getColumn("vasti"))
    maxGlut = np.max(vin[1].getColumn("glut_max"))
    maxHams = np.max(vin[1].getColumn("hamstrings"))
    maxIlpso = np.max(vin[1].getColumn("iliopsoas"))
    maxRecfem = np.max(vin[1].getColumn("recfem"))
    maxGas = np.max(vin[1].getColumn("glut_max"))
    print(maxVas, maxGlut, maxHams, maxIlpso, maxRecfem, maxGas)


# In[15]:


# peak com velocity
for vin in files:
    maxComVelY = np.max(vin[7].getColumn("com_vel_y"))
    print(maxComVelY)

