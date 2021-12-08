#include "termination.h"
#include <math.h>
#include "OpenSim/OpenSim.h"

TerminateSimulation::TerminateSimulation(const OpenSim::Model &osimModel, const double threshold) : 
        TriggeredEventHandler(SimTK::Stage::Position), osimModel(osimModel), threshold(threshold){
}

SimTK::Real TerminateSimulation::getValue(const SimTK::State &s) const {
    const SimTK::Vec3 comT = osimModel.getMatterSubsystem().calcSystemMassCenterLocationInGround(s);
    const double disTar = std::sqrt(std::pow(comT[0]-comTarget[0], 2) + std::pow(comT[1]-comTarget[1], 2));
    return disTar - threshold;
}

void TerminateSimulation::handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const {
    terminate = true;
}