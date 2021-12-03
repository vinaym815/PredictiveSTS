#include "termination.h"
#include <math.h>
#include "OpenSim/OpenSim.h"

TerminateSimulation::TerminateSimulation(const OpenSim::Model &osimModel, const double threshold) : 
        TriggeredEventHandler(SimTK::Stage::Position), osimModel(osimModel), threshold(threshold){
}

SimTK::Real TerminateSimulation::getValue(const SimTK::State &s) const {
    const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
    const double jointAngles = SimTK::convertRadiansToDegrees(fabs(coordSet.get("ankle_angle").getValue(s)) + 
                                                    fabs(coordSet.get("knee_angle").getValue(s)) + 
                                                    fabs(coordSet.get("hip_flexion").getValue(s))); 

    return jointAngles - threshold;
}

void TerminateSimulation::handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const {
    terminate = true;
    teminationTime = s.getTime();
}

double TerminateSimulation::getTerminationTime() const{
    return teminationTime;
}