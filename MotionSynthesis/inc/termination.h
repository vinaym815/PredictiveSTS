#pragma once
#include "OpenSim/OpenSim.h"
#include <string>

class TerminateSimulation: public SimTK::TriggeredEventHandler {
public:
  TerminateSimulation(const OpenSim::Model &osimModel, const double threshold=10.0);
  SimTK::Real getValue(const SimTK::State &s) const;
  void handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const;

private:
  const OpenSim::Model &osimModel;
  const double threshold;
};