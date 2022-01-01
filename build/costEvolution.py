import os
import sys
import numpy as np
from matplotlib import pyplot as plt

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

startGen = 1
endGen = 800
steps = 1

fileName = sys.argv[1]

costWeights = np.array([800, 6, 0.3, 1.2, 175, 70, 10, 1000, 0.1])
costLabels = [r"$\phi_1$", r"$\phi_9$", r"$\phi_{10}$", r"$\phi_2$", 
                r"$\phi_3$", r"$\phi_4$", r"$\phi_6$", r"$\phi_8$",
                r"$\phi_7$"]

colors = plt.cm.tab10(np.linspace(0, 1, 11))
# 1st generation is skipped as it has nan as cost value
logData = np.genfromtxt(fileName, delimiter=",", skip_header=1)
logData = logData[range(startGen, endGen, steps), :]
np.savetxt("filteredFile.csv", logData, delimiter=",")

# Removing the costs file if it already exists
if (os.path.exists("costsData.csv")):
    os.remove("costsData.csv")

# Evaluating the costs
numEvals = len(logData)
os.system(f"./replay filteredFile.csv 1 {numEvals} false false")
fileData = np.genfromtxt("costsData.csv", delimiter=",")

# Sorting based on function Evaluations
fileDataSorted = fileData[np.argsort(fileData[:, 0])]
progress = 100*(1 - fileDataSorted[:,2])

# Weighting the costs
(rows, cols) = fileDataSorted.shape
for i in range(cols-3):
    fileDataSorted[:,i+2] = costWeights[i]*fileDataSorted[:,i+2]

# Removing gens with replay cost error
replayCost = np.sum(fileDataSorted[:,2:-1], axis=1)
simulationCost = logData[:,1]
mask = []
for i in range(replayCost.shape[0]):
    if(abs(replayCost[i]-simulationCost[i])<20):
        mask.append(True)
    else:
        mask.append(False)

fileDataSorted = fileDataSorted[mask,:]
progress = progress[mask]

# Smoothing the individual costs
nPonitSmoothing= 10
(rows, cols) = fileDataSorted.shape
smoothedData = np.zeros((rows-nPonitSmoothing+1, cols))
for i in range(cols):
    smoothedData[:, i] = np.convolve(fileDataSorted[:, i], np.ones(nPonitSmoothing), "valid")/nPonitSmoothing

fileDataSorted = smoothedData
progress = np.convolve(progress, np.ones(nPonitSmoothing), "valid")/nPonitSmoothing

## Relative contributions to total costs
(rows, cols) = fileDataSorted.shape
contributions = np.zeros((rows, cols-3))
for i in range(cols-3):
    contributions[:,i] = np.divide(fileDataSorted[:,i+2], fileDataSorted[:,1])*100

# Plotting the evaluation of costs
gens = fileDataSorted[:,0]
plt.figure(figsize=(5.5, 3), dpi=300)
plt.plot(gens, fileDataSorted[:,1], "-", label=r"$\phi_{total}$", linewidth=1, color=colors[0], alpha=0.75)

plt.plot(gens, fileDataSorted[:,10], label=costLabels[8], color=colors[9], alpha=0.75)
plt.plot(gens, fileDataSorted[:,6], label=costLabels[4], color=colors[5], alpha=0.75)
plt.plot(gens, fileDataSorted[:,7], label=costLabels[5], color=colors[6], alpha=0.75)
plt.plot(gens, fileDataSorted[:,2], label=costLabels[0], color=colors[1], alpha=0.75)
plt.plot(gens, fileDataSorted[:,3], label=costLabels[1], color=colors[2], alpha=0.75)
plt.plot(gens, fileDataSorted[:,4], label=costLabels[2], color=colors[3], alpha=0.75)
plt.plot(gens, fileDataSorted[:,9], label=costLabels[7], color=colors[8], alpha=0.75)
plt.plot(gens, fileDataSorted[:,8], label=costLabels[6], color=colors[7], alpha=0.75)
plt.plot(gens, fileDataSorted[:,5], label=costLabels[3], color=colors[4], alpha=0.75)

plt.xlim(gens[0], gens[-1])
plt.xlabel(r"$Generations$")
plt.ylabel(r"$Value$")
#plt.legend(loc="upper right")
plt.savefig("CostsVsGen.png", format="png", transparent=False, bbox_inches='tight')

plt.figure(figsize=(5.5, 3), dpi=300)
plt.plot(gens, progress, color="k", label=r"$100\times\alpha$", alpha=0.75)
plt.plot(gens, contributions[:,8], label=costLabels[8], color=colors[9], alpha=0.75)
plt.plot(gens, contributions[:,4], label=costLabels[4], color=colors[5], alpha=0.75)
plt.plot(gens, contributions[:,5], label=costLabels[5], color=colors[6], alpha=0.75)
plt.plot(gens, contributions[:,0], label=costLabels[0], color=colors[1], alpha=0.75)
plt.plot(gens, contributions[:,1], label=costLabels[1], color=colors[2], alpha=0.75)
plt.plot(gens, contributions[:,2], label=costLabels[2], color=colors[3], alpha=0.75)
plt.plot(gens, contributions[:,7], label=costLabels[7], color=colors[8], alpha=0.75)
plt.plot(gens, contributions[:,6], label=costLabels[6], color=colors[7], alpha=0.75)
plt.plot(gens, fileDataSorted[:,3], label=costLabels[3], color=colors[4], alpha=0.75)

plt.xlim(gens[0], gens[-1])
plt.xlabel(r"$Generations$")
plt.ylabel(r"$Value$")
#plt.legend(loc="upper right")
plt.savefig("CostsContribVsGen.png", format="png", transparent=False, bbox_inches='tight')

plt.show()