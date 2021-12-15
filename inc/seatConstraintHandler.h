#pragma once
#include "OpenSim/OpenSim.h"
#include "universalConsts.h"

// Disables the seat kinematic constraint if the 

class ReleaseSeatConstraint : public SimTK::TriggeredEventHandler {
public:

  ReleaseSeatConstraint(OpenSim::Model &m, const SimTK::ConstraintIndex index, const double threshold);
  SimTK::Real getValue(const SimTK::State& s) const;
  void handleEvent(SimTK::State& s, SimTK::Real accuracy, bool& terminate) const;
  double getSeatRleaseTime() const ;

private:
  OpenSim::Model &_model;
  const double _forceThreshold;
  const SimTK::ConstraintIndex _index;
  mutable double seatReleaseTime = -1;
};