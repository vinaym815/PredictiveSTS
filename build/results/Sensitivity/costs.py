# Opens the file logBest.txt from all the subdirectories of current folder 
# and extracts the best values from second column

# %%

import os 
import numpy as np

# %%

nanValue = 1e10
directoryContents = os.listdir()
subDirectories = [directory for directory in directoryContents if os.path.isdir(directory)]

minimumCosts = []
for folder in subDirectories:
    fileName = folder + "/logBest.txt"
    file = np.genfromtxt(fileName, delimiter=",",)
    np.nan_to_num(file, copy=False, nan=nanValue)
    minimumCosts.append(np.min(file[:,1]))

# %%
costWeights = []
for folder in subDirectories:
    fileName = folder + "/comments.txt"
    file = np.genfromtxt(fileName, delimiter=",",)
    costWeights.append(file)


# %%
file = np.genfromtxt("../00pct/logBest.txt", delimiter=",")
np.nan_to_num(file, copy=False, nan=nanValue)
baseValue = np.min(file[:,1])
print(baseValue)

# %%

for weights, cost in zip(costWeights, minimumCosts):
    print(weights, cost)