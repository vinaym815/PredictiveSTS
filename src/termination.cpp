#include "termination.h"
#include <math.h>
#include "OpenSim/OpenSim.h"

TerminateSimulation::TerminateSimulation(const OpenSim::Model &osimModel, const double threshold) : 
        TriggeredEventHandler(SimTK::Stage::Position), osimModel(osimModel), threshold(threshold){
}

SimTK::Real TerminateSimulation::getValue(const SimTK::State &s) const {
    const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
    const double ankleAngle = coordSet.get("ankle_angle").getValue(s);
    const double kneeAngle = coordSet.get("knee_angle").getValue(s);
    const double hipAngle = coordSet.get("hip_flexion").getValue(s);

    const double tibiaAngleGround = ankleAngle;
    const double femurAngleGround = ankleAngle-kneeAngle;
    const double torsoAngleGround = ankleAngle-kneeAngle+hipAngle;

    const double jointAngles = SimTK::convertRadiansToDegrees(fabs(tibiaAngleGround) + 
                                                    fabs(femurAngleGround) + 
                                                    fabs(torsoAngleGround)); 

    return jointAngles - threshold;
}

void TerminateSimulation::handleEvent(SimTK::State &s, SimTK::Real accuracy, bool& terminate) const {
    terminate = true;
    teminationTime = s.getTime();
}

double TerminateSimulation::getTerminationTime() const{
    return teminationTime;
}