#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State &si0, const SimTK::State &siF, 
                          const double seatOffTime){

  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>(
                                                    &osimModel.updAnalysisSet().get("forceReporter"));

  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &forceStorage = frcReporter->getForceStorage();

  const SimTK::Vec4 feetCosts = computeCostFeet(osimModel, si0, seatOffTime);

  const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
  const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);

  const double d0 = eucledianDis(comT0, comTarget);
  const double df = eucledianDis(comTf, comTarget);

  const double progress = 1.0-std::min(d0,df)/d0;
  //std::cout << "progress " << progress << std::endl;

  costs[0] = df/d0;
  costs[1] = progress*computeCostJointVel(osimModel, siF);
  costs[2] = progress*feetCosts[3];
  costs[3] = (1.0-progress)*computeCostChair(forceStorage);
  costs[4] = computeCostActivation(activationTimeSeries);
  costs[5] = computeCostDiffActivation(activationTimeSeries);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = progress*feetCosts[0];
  costs[8] = progress*feetCosts[1];
  costs[9] = progress*feetCosts[2];
  #ifdef Assisted
    costs[10] = computeCostAssistance(forceStorage);
  #endif

  //// Clearing the reporters
  muscleActivReporter->clearTable();
};

double computeCostJointVel(const OpenSim::Model &osimModel, const SimTK::State &siF){
  const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
  const double hipVel = coordSet.get("hip_flexion").getSpeedValue(siF);
  const double kneeVel= coordSet.get("knee_angle").getSpeedValue(siF);
  const double ankleVel= coordSet.get("ankle_angle").getSpeedValue(siF);

  return SimTK::convertRadiansToDegrees(fabs(hipVel)+fabs(kneeVel)+fabs(ankleVel));
}

#endif