#!/usr/bin/env python
# coding: utf-8

# In[1]:


from matplotlib import pyplot as plt
import numpy as np 
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


# In[3]


nPoints = 100
startThreshold = np.deg2rad(20)
endThreshold = np.deg2rad(20)
simStartEndnPointMovingAvg = 1
colors = plt.cm.tab10(np.linspace(0, 1, 8))
muscleDict = {"soleus" :("SOL", "pink"),
              "gastroc":("GAS",colors[1]),
              "tibant":("TA", colors[2]),
              "recfem":("RF", colors[3]),
              "iliopsoas":("ILPSO",colors[4]),
              "hamstrings":("HAMS",colors[5]),
              "glut_max":("GMAX",colors[6]),
              "vasti":("VAS",colors[7])}


# In[3]:


def plotActuation(ax, file1, file2, file3, file4, file5, file6, file7):

    motionXAxis = np.linspace(0,100, nPoints)
    startTime, endTime = computeMotionTimeRange(("time", "hip_flexion"), (startThreshold, endThreshold), file7.fileName, 8,simStartEndnPointMovingAvg)

    timeData = file7.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    tEquallySpace = np.linspace(timeData[startInd], timeData[endInd], nPoints)
    
    (e1, e2) = computePhaseTimes(file2, file7)
    indE1 = np.argmax(tEquallySpace>=e1-1e-3)
    indE2 = np.argmax(tEquallySpace>=e2-1e-3)
    e1 = motionXAxis[indE1]
    e2 = motionXAxis[indE2]
    
    muscles = ["soleus", "gastroc", "tibant", "recfem", "iliopsoas", "hamstrings", "glut_max", "vasti"]
    for file_, axes in zip((file1, file2), ax):
        timeData = file_.getColumn("time")
        startInd = np.argmax(timeData>=startTime-1e-3)
        endInd = np.argmax(timeData>=endTime-1e-3)
        
        for i, muscle in enumerate(muscles):
            muscleData = np.interp(tEquallySpace, timeData[startInd:endInd], file_.getColumn(muscle)[startInd:endInd])
            axes.plot(motionXAxis, muscleData, label=muscleDict[muscle][0], color=muscleDict[muscle][1])
    
    muscles = ["iliopsoas", "recfem", "hamstrings", "glut_max"]
    timeData = file4.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)    
    for i, muscle in enumerate(muscles):
        muscleData = np.interp(tEquallySpace, timeData[startInd:endInd], file4.getColumn(muscle)[startInd:endInd])
        ax[2].plot(motionXAxis, muscleData, label=muscleDict[muscle][0], color=muscleDict[muscle][1])
    
    timeData = file3.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    momentData = np.interp(tEquallySpace, timeData[startInd:endInd], file3.getColumn("hip_flexion_moment")[startInd:endInd])
    ax[2].plot(motionXAxis, momentData, "--",color="green",linewidth=2, label="Hip")

    muscles = ["gastroc", "recfem", "hamstrings", "vasti"]
    timeData = file5.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)  
    for i, muscle in enumerate(muscles):
        muscleData = np.interp(tEquallySpace, timeData[startInd:endInd], file5.getColumn(muscle)[startInd:endInd])
        ax[3].plot(motionXAxis, muscleData, label=muscleDict[muscle][0], color=muscleDict[muscle][1])
    
    timeData = file3.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    momentData = np.interp(tEquallySpace, timeData[startInd:endInd], file3.getColumn("knee_angle_moment")[startInd:endInd])
    ax[3].plot(motionXAxis, momentData, "--",color="darkviolet",linewidth=2, label="Knee")
    
    muscles = ["gastroc", "soleus", "tibant"]
    timeData = file6.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    for i, muscle in enumerate(muscles):
        muscleData = np.interp(tEquallySpace, timeData[startInd:endInd], file6.getColumn(muscle)[startInd:endInd])
        ax[4].plot(motionXAxis, muscleData, label=muscleDict[muscle][0], color=muscleDict[muscle][1])
    
    timeData = file3.getColumn("time")
    startInd = np.argmax(timeData>=startTime-1e-3)
    endInd = np.argmax(timeData>=endTime-1e-3)
    momentData = np.interp(tEquallySpace, timeData[startInd:endInd], file3.getColumn("ankle_angle_moment")[startInd:endInd])
    ax[4].plot(motionXAxis, momentData, "--",color="deepskyblue",linewidth=2, label="Ankle")
    
    limits = [(-0.05, 1.05), (0.0, 16000), (-175, 120), (-320, 110), (-151, 150)]
    
    for axes, limit, i in zip(ax, limits, range(len(limits))):
        axes.plot([e1,e1], limit, linestyle='-.', color="k", alpha=0.3)
        axes.plot([e2,e2], limit, linestyle='-.', color="k", alpha=0.3)
        
        #handlelength=2.5
        #loc = "lower right"        
        #if i==0:
        #    handlelength=1
        #if (i== 0 or i ==1):
        #    loc = "upper right"
            
        #handles, labels = axes.get_legend_handles_labels()
        #axes.legend(handles[::-1], labels[::-1], loc=loc, fontsize = 6, handlelength=handlelength)

    return None


# In[4]:


models = ["00pct", "20pct", "40pct", "60pct", "80pctAssisted"]

files = []
for model in models:
    
    yDataNames = ["gastroc", "glut_max", "iliopsoas", "recfem", "tibant", "hamstrings", "soleus","tibant", "vasti"]
    regrexHeader = r"(\w+)/activation$"
    file1 = file(yDataNames, "../ProcessedData/"+model+"/"+model+".sto", 8, regrexHeader=regrexHeader)
                 
    yDataNames = ["gastroc", "glut_max", "iliopsoas", "recfem", "tibant", "hamstrings", "soleus","tibant", "vasti", 
                  "seatConstraint_ground_Fy"]
    file2 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_force.mot", 14)
    
    yDataNames = ["ankle_angle_moment", "knee_angle_moment", "hip_flexion_moment"]
    file3 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_ID.sto", 6)
    
    yDataNames = ["recfem", "hamstrings", "glut_max", "iliopsoas"]
    file4 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_ID_Hip.sto", 11)
    
    yDataNames = ["gastroc", "recfem", "hamstrings", "vasti"]
    file5 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_ID_Knee.sto", 11)
    
    yDataNames = ["gastroc", "soleus", "tibant"]
    file6 = file(yDataNames, "../ProcessedData/"+model+"/"+model+"_ID_Ankle.sto", 11)
        
    yDataNames = ["hip_flexion"]
    file7 = file(yDataNames, "../ProcessedData/"+model+"/"+model+".sto", 8)
    
    files.append((file1, file2, file3, file4, file5 ,file6, file7))


# In[5]:


fig, axs = plt.subplots(5, 4, sharex='col', sharey='row', figsize=(8.2, 8), dpi=300)

plotActuation(axs[:,0], *files[0])
plotActuation(axs[:,1], *files[1])
plotActuation(axs[:,2], *files[2])
plotActuation(axs[:,3], *files[3])

limits = [(-0.05, 1.05), (0.0, 6000), (-85, 50), (-150, 100), (-35, 50)]

for i in range(4):
    axs[4,i].set_xlim([0, 100])

for i,limit in enumerate(limits):
    axs[i,0].set_ylim(limit)

fig.text(0.5, -0.01, '\% STS', ha='center', fontsize=10)
fig.text(-0.005, 0.12, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.31, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.5, 'Torques (Nm)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.72, 'Force (N)', va='center', rotation='vertical', fontsize=10)
fig.text(-0.005, 0.9, 'Activation', va='center', rotation='vertical', fontsize=10)

fig.text(0.1, 1, '0\% Strength Deficit', fontsize=10)
fig.text(0.32, 1, '20\% Strength Deficit', fontsize=10)
fig.text(0.55, 1, '40\% Strength Deficit', fontsize=10)
fig.text(0.8, 1, '60\% Strength Deficit', fontsize=10)

fig.set_tight_layout(True)

#plt.savefig("figures/Actuation.png", format="png",transparent=False, bbox_inches = 'tight')
#plt.show()


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

print("Peak Knee Torque, contributions: vas, hams, rf, gas, t")
for vin in files:
    print(peakTorqueKnee(vin[2], vin[4]))


# In[18]:

    
print("Peak Hip Torque, contributions: gmax, hams, rf, ilipso, t")
for vin in files:
    print(peakTorqueHip(vin[2], vin[3]))


# In[19]:


# time difference peak hip and knee torque
for vin in files:
    peakKnee = peakTorqueKnee(vin[2], vin[4])
    peakHip = peakTorqueHip(vin[2], vin[3])
    print("dt Peak (Hip-Knee) torque : ", peakKnee[-1] - peakHip[-1])


# In[9]:


# Maximum contribution to knee torque
for vin in files:
    peakVasKnee = np.min(vin[4].getColumn("vasti"))
    peakHamsKnee = np.max(vin[4].getColumn("hamstrings"))
    peakRfKnee = np.min(vin[4].getColumn("recfem"))
    peakGasKnee = np.min(vin[4].getColumn("gastroc"))
    print("peak contibutions to  knee torques : vas, hams, knee, gas")
    print((peakVasKnee, peakHamsKnee,peakRfKnee, peakGasKnee))


# In[10]:


# Maximum contribution to hip torque
for vin in files:
    peakGMaxHip = np.min(vin[3].getColumn("glut_max"))
    peakHamsHip = np.min(vin[3].getColumn("hamstrings"))
    peakRfHip = np.min(vin[3].getColumn("recfem"))
    peakIlipsoHip = np.min(vin[3].getColumn("iliopsoas"))
    print("peak contributions to hip torque: gmax, hams, rf, ilpso")
    print((peakGMaxHip, peakHamsHip, peakRfHip, peakIlipsoHip))


# In[11]:


for vin in files:
    print("peak ankle moment : ", np.min(vin[2].getColumn("ankle_angle_moment")))


# In[13]:


# Maximum muscle forces
print("peak muscle forces : vas, glut, hams, ilpso, rf, gas")
for vin in files:
    maxVas = np.max(vin[1].getColumn("vasti"))
    maxGlut = np.max(vin[1].getColumn("glut_max"))
    maxHams = np.max(vin[1].getColumn("hamstrings"))
    maxIlpso = np.max(vin[1].getColumn("iliopsoas"))
    maxRecfem = np.max(vin[1].getColumn("recfem"))
    maxGas = np.max(vin[1].getColumn("gastroc"))
    

    print(maxVas, maxGlut, maxHams, maxIlpso, maxRecfem, maxGas)