#pragma once
#include <iostream>
#include <string>
#include "cmaes.h"
#include "OpenSim.h"

#define Standing                            // Commenting will switch to Sitting
//#define Assisted                            // Commenting will switch to unAssisted
//#define Debug

// Sets up model and number of variables
#ifdef Assisted
  const std::string modelName = "vin_60pctAssisted.osim";
  const std::string newModelName = "vin_60pctAssisted.osim.mod";
  const std::string newModelNameReplay = "vin_60pctAssisted_replay.osim.mod";

  const int numExtFuncs = 10;                  // Number of unique excitation functions
  #ifdef Standing
    const size_t numWeights = 10;
  #else
    const size_t numWeights = 8;
  #endif

#else
  const std::string modelName = "vin_00Npct.osim";
  const std::string newModelName = "vin_00Npct.osim.mod";
  const std::string newModelNameReplay = "vin_00Npct_replay.osim.mod";
  //const std::string modelName = "vin_00pct.osim";
  //const std::string newModelName = "vin_00pct.osim.mod";
  //const std::string newModelNameReplay = "vin_00pct_replay.osim.mod";
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
      const size_t numWeights = 9;
  #else
      const size_t numWeights = 7;
  #endif
#endif

// Simulation Setup
const double t0 = 0.0;             // Simulation starting time
const double T_MAX = 1.6;

const double MU_STATIC = 0.8;               // coefficient of static friction
const double SAMPLING_DT = 0.01;             // The rate at which excitation signal is sampled(sec) from the parameterization function
const double REPORT_INTERVAL = 0.001;         // Reporting Interval of table reporters(secs). May be made noisy to avoid peculiarities
const double INTEGRATOR_ACCURACY = 1.0e-4;   // Desired Integration Accuracy
const double DEFAULT_EXCITATION = 0.05;

// Optimization Setup 
const SimTK::Vec3 comTarget{0.0346674, 0.943719, -0.0546681};
const int NUM_COMPS = 16;                   // Number of parameterization componenets each excitation trajectory is made up of
const double TAU_CHAIR_FORCE = T_MAX/8.0;
const int N_RESTARTS = 3;
const double F_TOLERANCE = 0.5;
const int HIST_SIZE = 250;
const int MAX_ITER = 4000;
const std::string LOWER_BOUNDS_FILE = "lowerBound.txt";
const std::string UPPER_BOUNDS_FILE = "upperBound.txt";
const std::string STD_DEV_FILE = "stdDev.txt";

// Log file Names
const std::string logBestFileName = "logBest.txt";
const std::string logMeanFileName = "logMean.txt";
const std::string resumeMeanFileName = "resumeMean.dat";
const std::string resumeCovFileName = "resumeCov.dat";
const std::string resumeSigmaFileName = "resumeSigma.dat";

typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,libcmaes::linScalingStrategy> GenoPhenoType;