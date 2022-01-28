# %%
import os
import sys
import numpy as np
from matplotlib import pyplot as plt

# %%
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

# %%
startGen = 1
endGen = 800
steps = 1
replayErrorThreshold = 20
nPonitSmoothing= 10

fileName = "results/00pct/logBest.txt"
costWeights = np.array([800, 6, 0.3, 1.2, 175, 70, 10, 1000, 0.1])
costLabels = [r"$\phi_1$", r"$\phi_9$", r"$\phi_{10}$", r"$\phi_2$", 
                r"$\phi_3$", r"$\phi_4$", r"$\phi_6$", r"$\phi_8$",
                r"$\phi_7$"]

colors = plt.cm.tab10(np.linspace(0, 1, 11))

# %%

# 1st generation is skipped as it has nan as the cost value
logData = np.genfromtxt(fileName, delimiter=",", skip_header=1)
logData = logData[range(startGen, endGen, steps), :]
np.savetxt("filteredFile.csv", logData, delimiter=",")

if (os.path.exists("costsData.csv")): os.remove("costsData.csv")

numEvals = len(logData)
os.system(f"./replay filteredFile.csv 1 {numEvals} false false")


# %% Reads the costs data, sorts it, weighs it, filters it based on replay error
# and smooths it using nPoint Moving Average

fileData = np.genfromtxt("costsData.csv", delimiter=",")
fileDataSorted = fileData[np.argsort(fileData[:, 0])]
progress = 100*(1 - fileDataSorted[:,2])

(rows, cols) = fileDataSorted.shape
for i in range(cols-3):
    fileDataSorted[:,i+2] = costWeights[i]*fileDataSorted[:,i+2]

replayCost = np.sum(fileDataSorted[:,2:-1], axis=1)
simulationCost = logData[:,1]
mask = [bool(replayCost[i] - simulationCost[i] < replayErrorThreshold) for i in range(len(replayCost))]

fileDataSorted = fileDataSorted[mask,:]
progress = progress[mask]

(rows, cols) = fileDataSorted.shape
smoothedData = np.zeros((rows-nPonitSmoothing+1, cols))
for i in range(cols):
    smoothedData[:, i] = np.convolve(fileDataSorted[:, i], np.ones(nPonitSmoothing), "valid")/nPonitSmoothing

fileDataSorted = smoothedData
progress = np.convolve(progress, np.ones(nPonitSmoothing), "valid")/nPonitSmoothing

(rows, cols) = fileDataSorted.shape
contributions = np.zeros((rows, cols-3))
for i in range(cols-3):
    contributions[:,i] = np.divide(fileDataSorted[:,i+2], fileDataSorted[:,1])*100

# %% Plotting 
gens = fileDataSorted[:,0]

plottingOrder = [8, 4, 5, 0, 1, 2, 7, 6 ,3]
fig,axs = plt.subplots(2, 1, sharex=True, figsize=(6,4), dpi=300)

axs[0].plot(gens, fileDataSorted[:,1], "-", label=r"$\phi_{total}$", linewidth=1, color=colors[0], alpha=0.75)
axs[1].plot(gens, progress, color="k", label=r"$100\times\alpha$", alpha=0.75)
for i in plottingOrder:
    axs[0].plot(gens, fileDataSorted[:,i+2], color=colors[i+1], label=costLabels[i], alpha=0.75)
    axs[1].plot(gens, contributions[:,i], color=colors[i+1], label=costLabels[i], alpha=0.75)


axs[1].set_xlim(gens[0], gens[-1])
axs[0].set_ylabel("Value")
axs[1].set_ylabel("Value")
axs[1].set_xlabel("Generations")
#axs[0].legend(loc="upper right")
#axs[1].legend(loc="upper right")

plt.savefig("CostsVsGen.png", format="png", transparent=False, bbox_inches='tight')
plt.show()
# %%
