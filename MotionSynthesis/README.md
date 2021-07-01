We use single shooting optimization to generate Assisted/UnAssisted Sit-to-stand/Sitting trajectories.

The Assisted/UnAssisted Sitting trajectories are used as intial guesses to Assisted/UnAssisted Sit-to-Stand optimizations.

The assistance is provided at the torso's center of mass.

We use aCMAES for optimization.
The optimization tunes the excitation signals to the muscles and external assistance forces.

The Standing and Assisted flags in the universalConsts.h file decides wheter the optimization is of Standing/Sitting
or Assisted/UnAssisted trajectories

universalConsts.h and hyperparams___.txt contains the hyperparameters

CMake can be use to build tht main.cpp or replay.cpp
main.cpp contains the optimization code for generating Assisted/Unassisted Sit-to-Stand or Sitting trajectory

commands for running the optimization/visualization:

./main initialGuessFileName.txt hyperParamsFileName.txt resumeDir(null/dir)
./replay modelName.osim fileName.txt genStart genEnd log(true/false)

file naming convention:

If a model name consists of "Assist" it has external assistance acting on it
"_\*\*pct" in a file's name means it intended for use with model having maximum isometric strength \*\* percent of normal model. 