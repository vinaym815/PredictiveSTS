import os
import sys
import numpy as np
from matplotlib import pyplot as plt

# Setting up the plotting style
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

startGen = 0
endGen = 2222
steps = 10
nPonitSmoothing = 10

fileName = sys.argv[1]

costWeights = np.array([400, 0.1, 0.4, 0.4, 100, 20, 10, 20, 0.0, 0.1])
costLabels = [r"Progress", r"Vel_f", r"F_{feet}", r"F_{chair}", 
                r"a", r"$\dot{a}$", r"T_{Limit}", r"ZMP",
                r"UniLa", r"Slip"]

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
os.system(f"./replay 16 filteredFile.csv 1 {numEvals} false false")
fileData = np.genfromtxt("costsData.csv", delimiter=",")

# Sorting based on function Evaluations
fileDataSorted = fileData[np.argsort(fileData[:, 0])]

# Smoothing the individual costs
(rows, cols) = fileData.shape
smoothedData = np.zeros((rows-nPonitSmoothing+1, cols))
for i in range(cols):
    smoothedData[:, i] = np.convolve(fileDataSorted[:, i], np.ones(nPonitSmoothing), "valid")/nPonitSmoothing

progress = 100*(-smoothedData[:,2] + 1)
# Weighting the costs
for i in range(cols-3):
    smoothedData[:,i+2] = costWeights[i]*smoothedData[:,i+2]

## Relative contributions to total costs
contributions = np.zeros((rows-nPonitSmoothing+1, cols-3))
for i in range(cols-3):
    contributions[:,i] = np.divide(smoothedData[:,i+2], smoothedData[:,1])*100

# Plotting the evaluation of costs
gens = smoothedData[:,0]
plt.figure(figsize=(8.5, 5.5), dpi=300)
plt.plot(gens, smoothedData[:,1], "-", label="TotalCostSimulation", linewidth=2, color=colors[0])

#replayCost = []
#for i in range(smoothedData.shape[0]):
#    replayCost.append(np.dot(costWeights, smoothedData[i,2:-1]))
#
#plt.plot(gens, replayCost, "--", label="TotalCostReplay", linewidth=2)

for i in range(costWeights.size):
    plt.plot(gens, smoothedData[:,i+2], label=costLabels[i], color=colors[i+1])

plt.xlim(gens[0], gens[-1])
plt.xlabel("Generations ")
plt.ylabel("Value")
plt.legend()
plt.savefig("CostsVsGen.png", format="png", transparent=False, bbox_inches='tight')

plt.figure(figsize=(8.5, 5.5), dpi=300)
plt.plot(gens, np.divide(smoothedData[:,1], smoothedData[:,1])*100, label="TotalCost", color=colors[0])

for i in range(contributions.shape[1]):
    plt.plot(gens, contributions[:,i], label=costLabels[i], color=colors[i+1])
plt.plot(gens, progress, label=r"$\%STS$")

plt.xlim(gens[0], gens[-1])
plt.xlabel("Generations ")
plt.ylabel("%")
plt.legend()
plt.savefig("CostsContribVsGen.png", format="png", transparent=False, bbox_inches='tight')

plt.show()