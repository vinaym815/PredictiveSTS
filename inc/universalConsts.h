#pragma once
#include <iostream>
#include <string>
#include "cmaes.h"

#define Standing                            // Commenting will switch to Sitting
//#define Assisted                            // Commenting will switch to unAssisted

// Sets up model and number of variables
#ifdef Assisted
  const std::string modelName = "vin_60pctAssisted.osim";
  const std::string newModelName = "vin_60pctAssisted.osim.mod";
  const std::string newModelNameReplay = "vin_60pctAssisted_replay.osim.mod";

  const int numExtFuncs = 10;                  // Number of unique excitation functions
  #ifdef Standing
    const size_t numWeights = 11;
    const int numHyperParams = 4+numWeights;
  #else
    const size_t numWeights = 9;
    const int numHyperParams = 4+numWeights;
  #endif

#else
  const std::string modelName = "vin_00pct.osim";
  const std::string newModelName = "vin_00pct.osim.mod";
  const std::string newModelNameReplay = "vin_00pct_replay.osim.mod";
  //const std::string modelName = "vin_20pct.osim";
  //const std::string newModelName = "vin_20pct.osim.mod";
  //const std::string newModelNameReplay = "vin_20pct_replay.osim.mod";
  //const std::string modelName = "vin_40pct.osim";
  //const std::string newModelName = "vin_40pct.osim.mod";
  //const std::string newModelNameReplay = "vin_40pct_replay.osim.mod";
  //const std::string modelName = "vin_60pct.osim";
  //const std::string newModelName = "vin_60pct.osim.mod";
  //const std::string newModelNameReplay = "vin_60pct_replay.osim.mod";

  const int numExtFuncs = 8;                  // Number of unique excitation functions
  #ifdef Standing
      const size_t numWeights = 10;
      const int numHyperParams = 4+numWeights;
  #else
      const size_t numWeights = 8;
      const int numHyperParams = 4+numWeights;
  #endif
#endif

// Cost Function Setup
#ifdef Standing
    const double tau_ChairForce_pct = 1.0/3.0;
    const double tau_Boundary_pct = 1.0/24.0;
#endif

// Simulation Setup
const double mu_static = 0.8;               // coefficient of static friction
const double simulationDuration = 1.6;
const double samplingDt = 0.01;             // The rate at which excitation signal is sampled(sec) from the parameterization function
const double reportInterval = 0.001;         // Reporting Interval of table reporters(secs). May be made noisy to avoid peculiarities
const double integratorAccuracy = 1.0e-4;   // Desired Integration Accuracy
const double initialTime = 0.0;             // Simulation starting time
const double defaultExcitationController = 0.05;

// Optimization Setup 
const int nRestarts = 3;
const double fTolerance = 0.5;
const int histSize = 250;
const int maxIter = 7000;

// Log file Names
const std::string logBestFileName = "logBest.txt";
const std::string logMeanFileName = "logMean.txt";
const std::string resumeMeanFileName = "resumeMean.dat";
const std::string resumeCovFileName = "resumeCov.dat";
const std::string resumeSigmaFileName = "resumeSigma.dat";

typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,libcmaes::linScalingStrategy> GenoPhenoType;