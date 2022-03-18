# Sit-to-Stand Motion Synthesis

https://user-images.githubusercontent.com/35987596/158946265-1d420c43-7786-4519-89ab-19d82669cf08.mp4

## Overview
Sit-to-stand (STS) transition is one of the most biomechanically challenging task necessary for performing activities of daily life.
Many physiological and psychological factors, with muscle strength being the most dominant, affect how an individual performs STS transition.
Many co-occurring factors make it complex to establish cause-effect relationships using experiments.
We use this code to predict the effects of muscle weakness on STS and predict the assistance trajectories that can complement muscle weakness for assisted STS. 
The muscle weakness is introduced by scaling the maximum isometric strength of the Hill Type Muscles.
The assistance is provided at the torso's centre of mass.

The code tunes the open-loop excitation trajectories of muscles and external assistance using single shooting optimization to generate STS trajectories.
Active Covariance Matrix Adaptation Evolution Strategy (aCMAES) is the optimization algorithm.
Sitting muscle excitation trajectories, also generated using the same framework, are used as initial guesses (mean) to the optimization framework.
Please refer to the code or docs for more information on the cost functions used to generate STS motions.

## Building Details
1. Set up the *Standing* and *Assisted* flags, model name and other hyper-parameters in the universalConsts.h file.
2. Use the CMakeLists to build either the *main.cpp* or *replay.cpp*. *main.cpp* contains the optimization code for generating Assisted/Unassisted Sit-to-Stand or Sitting trajectories.
3. Edit the optimization parameter files: initial guess, upper bound, lower bound and standard deviation. 
4. Run the *./main initialGuessFileName.txt weightsFile.txt resumeDir(null/dir)* or *./replay logfileName.txt genStart genEnd visualize(true/false) saveResults(true/false)* commands. 

- The main file outputs logs in build/results/FolderName/ directory
- For resuming the optimization, the directory path containing the log file directory should be passed to the function. 
- Replay file can be used with the logBest.txt or logMean.txt files for visualization

### Dependencies:
1. libcmaes (https://github.com/CMA-ES/libcmaes/tree/2c2f56068c562a3967fe7ccb4b42e48dd1d7be83)
2. opensim-core (https://github.com/opensim-org/opensim-core/releases/tag/4.3) 

## File Naming Convention
- **"Assisted/UnAssisted"** is used within the file names to indicate whether it is supposed to work or not with external assistance.
- **"_\*\*pct"** percentage maximum isometric strength deficit. *0%* corresponds to the maximum isometric strength of a healthy adult

## Citation
Kumar, V., Yoshiike, T., & Shibata, T.\
Predicting sit-to-stand adaptations due to muscle strength deficits and assistance trajectories to complement them.\
Frontiers in Bioengineering and Biotechnology, 330.\
https://doi.org/10.3389/fbioe.2022.799836
