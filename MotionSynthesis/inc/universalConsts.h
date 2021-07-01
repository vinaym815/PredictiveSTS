#pragma once
#include <iostream>
#include <string>
#include "cmaes.h"
#include "activationPointActuator.h"

#define Standing                            // Commenting will switch to Sitting
#define Assisted                            // Commenting will switch to unAssisted

#ifdef Assisted
  const std::string modelName = "vinAssist_40pct.osim";
  const std::string newModelName = "vinAssist_40pct.osim.mod";
  const int numExtFuncs = 10;                  // Number of unique excitation functions
  #ifdef Standing
    const size_t numWeights = 10;
    const int numHyperParams = 7+2*numWeights;
  #endif
  #ifndef Standing
    const size_t numWeights = 9;
    const int numHyperParams = 7+2*numWeights;
  #endif
#endif

#ifndef Assisted
  const std::string modelName = "vin_100pct.osim";
  const std::string newModelName = "vin_100pct.osim.mod";
  const int numExtFuncs = 8;                  // Number of unique excitation functions
  #ifdef Standing
      const size_t numWeights = 9;
      const int numHyperParams = 7+2*numWeights;
  #endif
  #ifndef Standing
      const size_t numWeights = 8;
      const int numHyperParams = 7+2*numWeights;
  #endif
#endif

#ifdef Standing
    const double tau_ChairForce_pct = 1.0/3;
    const double tau_Boundary_pct = 1.0/8;
#endif

const double mu_static = 0.8;               // coefficient of static friction

const double simulationDuration = 1.6;
const double samplingDt = 0.01;             // The rate at which excitation signal is sampled(sec) from the parameterization function
const double reportInterval = 0.001;         // Reporting Interval of table reporters(secs). May be made noisy to avoid peculiarities
const double convTolerance = 1e-2;          // Convergance Tolerance for optimization
const double integratorAccuracy = 1.0e-4;   // Desired Integration Accuracy
const double initialTime = 0.0;             // Simulation starting time
const double defaultExcitationController = 0.05;

// Log file Names
const std::string logBestFileName = "logBest.txt";
const std::string logMeanFileName = "logMean.txt";
const std::string resumeMeanFileName = "resumeMean.dat";
const std::string resumeCovFileName = "resumeCov.dat";
const std::string resumeSigmaFileName = "resumeSigma.dat";

typedef libcmaes::GenoPheno<libcmaes::pwqBoundStrategy,libcmaes::linScalingStrategy> GenoPhenoType;