#pragma once
#include <string>
#include "OpenSim/OpenSim.h"
#include "universalConsts.h"

// Terminates the simulation if the sum of absolute joint angles (degrees) goes below a certain threshold

class TerminateSimulation: public SimTK::TriggeredEventHandler {
public:
  TerminateSimulation(const OpenSim::Model &osimModel, const double threshold=1e-3);
  SimTK::Real getValue(const SimTK::State &s) const;
  void handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const;

private:
  const OpenSim::Model &osimModel;
  const double threshold;
};