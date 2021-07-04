#pragma once
#include "OpenSim/OpenSim.h"
#include <string>

// Terminates the simulation if the sum of absolute joint angles (degrees) goes below a certain threshold

class TerminateSimulation: public SimTK::TriggeredEventHandler {
public:
  TerminateSimulation(const OpenSim::Model &osimModel, const double threshold=10.0);
  SimTK::Real getValue(const SimTK::State &s) const;
  void handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const;

private:
  const OpenSim::Model &osimModel;
  const double threshold;
};