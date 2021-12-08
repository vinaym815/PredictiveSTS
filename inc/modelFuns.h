#pragma once
#include "OpenSim/OpenSim.h"
#include "parameterization.h"
#include "universalConsts.h"
#include "Eigen/Dense"
#include "seatConstraintHandler.h"
#include "termination.h"
#include "assistanceForceVisualization.h"
#include "activationPointActuator.h"
#include <memory>
#include <sstream>
#include <math.h>
#include <array>
#include <sys/stat.h>
#include <algorithm>
#include <random>
#include "iomanip"

using namespace libcmaes;

// Add the controller to the model and saves a dummy copy of it
// The dummy copy is used to avoid conflict with the replay file
void addComponentsToModel(const std::string modelName, const std::string newModelName, const double tUpperBound);
void addReporters(OpenSim::Model &osimModel);
void addController(OpenSim::Model &osimModel, const double tUB);

// Used to run a forward simulation
void setExcitations(OpenSim::Model &osimModel, SimTK::State &si0, const ParameterizationType parameterization,const int numComps, const double *compValues);
std::vector<double> runSimulation(OpenSim::Model &osimModel, const ParameterizationType &parameterization, const double *controls, const int numComps,
                                  const bool visualizeResults=false, const bool saveResults=false,
                                  const std::string="export");

// The model reference is not const as we need to clear the reporter tables
#ifdef Standing

  void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State &si0, const SimTK::State &siF,
                            const double seatReleaseTime);
  SimTK::Vec2 computeCostsChair(const std::vector<double> &weightVec, const OpenSim::Storage &forceStorage);
  double computeCostFeetForce(const std::vector<double> &weightVec,const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, 
                              const double bodyWeight);
  double bestEucledianDisToTarget(const OpenSim::TimeSeriesTableVec3 &comTimeSeries);

  // get exponentially increase weight 
  inline double getIncExpWeight(const double tConst, const double t, const double tF){
      return std::pow(SimTK::E,t/tConst)/(tConst*(std::pow(SimTK::E, tF/tConst) - 1));
  };

  // get exponentially decreasing weight 
  inline double getDecExpWeight(const double tConst, const double t, const double tF){
      return std::pow(SimTK::E,-t/tConst)/(tConst*(1-std::pow(SimTK::E, -tF/tConst)));
  };

  //// get epnential increasing weight vector
  std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double tF);
#else
  void computeCostsSitting(OpenSim::Model &osimModel, const SimTK::State si0, std::vector<double> &costs);
  SimTK::Vec2 computeCostsCoordinate(const OpenSim::TimeSeriesTable &coordTimeSeries);
#endif

//// Computes the individual costs
double computeCostActivation(const OpenSim::TimeSeriesTable &activationTimeSeries, const double chairContactLossTime);
double computeCostDiffActivation(const OpenSim::TimeSeriesTable &activationTimeSeries, const double chairContactLossTime);
double computeCostLimitTorque(const OpenSim::Storage &forceStorage);
SimTK::Vec3 computeCostFeet(const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, const SimTK::Vec3 heelPos, const SimTK::Vec3 toesPos);
#ifdef Assisted
  double computeCostAssistance(const OpenSim::Storage &forceStorage);
#endif

// Default : Read the minimum (#Total lines in file , 1000 Lines)
void readVector(std::vector<double> &vecOutput, std::string fileName, const int startLine=1, const int endLine=1000, const int offset=0);
void writeVector(const std::vector<double> x, const std::string fileName, std::ios_base::openmode writingMode= std::ios::app);
std::string createUniqueFolder(const std::string path);

//// cmaes functions
void progressFunc(const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols, const std::string logFolder);
CMASolutions resumeDistribution(const std::string dirName, CMAParameters<GenoPhenoType> &cmaparams);

// return adjacent_difference of a vector
template <typename V>
inline std::vector<double> dVector(V &vec){
  std::vector<double> outVec(vec.size());
  std::adjacent_difference(vec.begin(), vec.end(), outVec.begin());
  outVec[0] = 0.0;
  return outVec;
}

inline double eucledianDis(SimTK::Vec3 v1, SimTK::Vec3 v2){
  return std::sqrt(std::pow(v1[0]-v2[0], 2) + std::pow(v1[1]-v2[1], 2));
};