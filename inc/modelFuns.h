#pragma once

/*
Declaration of functions used to 
1) run the simulation
2) compute the costs
3) read and write logs
*/

#include <memory>
#include <math.h>
#include <algorithm>

#include "OpenSim/OpenSim.h"
#include "parameterization.h"
#include "universalConsts.h"
#include "Eigen/Dense"
#include "seatConstraintHandler.h"
#include "assistanceForceVisualization.h"
#include "activationPointActuator.h"

/*
Adds an feeforward controller to the models and saves a copy of it.
The copy is used to run simulation to avoid conflicts
*/
void addComponentsToModel(const std::string modelName, const std::string newModelName);
void addController(OpenSim::Model &osimModel);

// Adds different reporters used to compute costs
void addReporters(OpenSim::Model &osimModel);

// Sets up the actuator excitation trajectories for forward simulation
// by sampling the node points from parameterization.
// Also sets up a0=u0
void setExcitations(OpenSim::Model &osimModel, SimTK::State &si0,
                    const ParameterizationType parameterization,
                    const double *compValues);

// Runs a forward simulation and returns the costs
std::vector<double> runSimulation(OpenSim::Model &osimModel, const ParameterizationType &parameterization, 
                                const int numDecisionVars,const double *controls, 
                                const bool visualizeResults=false, const bool saveResults=false, 
                                const std::string="fileNamePrefix");

#ifdef Standing
void computeCostsStanding(std::vector<double> &costsOutputVec, OpenSim::Model &osimModel, const SimTK::State &si0, 
                          const SimTK::State &siF, const double seatOffTime);

double computeCostJointVel(const OpenSim::Model &osimModel, const SimTK::State &siF);
double computeCostChair(const OpenSim::Storage &forceStorage);

// Returns the eucledian distance between two vectors
inline double eucledianDis(const SimTK::Vec3 v1, const SimTK::Vec3 v2){
    return std::sqrt(std::pow(v1[0]-v2[0], 2) + std::pow(v1[1]-v2[1], 2));
};

// Returns the exponentially weighted values for timeVec 
std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double T);

// get exponentially increase weight 
inline double getExpWeight(const double tau, const double t, const double T){
    return std::pow(SimTK::E,t/tau)/(tau*(std::pow(SimTK::E, T/tau) - 1));
};

#else
void computeCostsSitting(std::vector<double> &costsOutputVec, OpenSim::Model &osimModel, const SimTK::State si0);
SimTK::Vec2 computeCostsCoordinate(const OpenSim::TimeSeriesTable &coordTimeSeries);
#endif

// Computes the cost associated with actuator activation
double computeCostActivation(const OpenSim::TimeSeriesTable &activationTimeSeries);

// Computes the cost associated with rate of change of actuator activation
double computeCostDiffActivation(const OpenSim::TimeSeriesTable &activationTimeSeries);

// Computes the penalty for using the mechanical limits at joints
double computeCostLimitTorque(const OpenSim::Storage &forceStorage);

// Returns zmp, slip and (Ffeet-mg) costs
SimTK::Vec3 computeCostFeet(OpenSim::Model &model, const SimTK::State &si0, const double seatOffTime);

#ifdef Assisted
// Computes the cost associated with external assistance
double computeCostAssistance(const OpenSim::Storage &forceStorage);
#endif

// returns the adjacent_difference of vector V
template <typename V>
inline std::vector<double> dVector(V &vec){
    std::vector<double> outVec(vec.size());
    std::adjacent_difference(vec.begin(), vec.end(), outVec.begin());
    outVec[0] = 0.0;
    return outVec;
}