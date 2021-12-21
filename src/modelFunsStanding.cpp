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

  const SimTK::Vec3 feetCosts = computeCostFeet(osimModel, si0, seatOffTime);

  const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
  const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);

  const double d0 = eucledianDis(comT0, comTarget);
  const double df = eucledianDis(comTf, comTarget);

  const double progress = 1.0-std::min(d0,df)/d0;
  //std::cout << "progress " << progress << std::endl;

  costs[0] = df/d0;
  costs[1] = progress*computeCostJointVel(osimModel, siF);
  costs[2] = progress*feetCosts[2];
  costs[3] = (1.0-progress)*computeCostChair(forceStorage);
  costs[4] = computeCostActivation(activationTimeSeries);
  costs[5] = computeCostDiffActivation(activationTimeSeries);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = progress*feetCosts[0];
  costs[8] = progress*feetCosts[1];
  #ifdef Assisted
    costs[9] = computeCostAssistance(forceStorage);
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

//// Can not convert to timeseries as it will break the storage due to constraint being disabled
double computeCostChair(const OpenSim::Storage &forceStorage){
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());
  const std::vector<double> dtVec = dVector(tVec);

  //// Force applied by the ground to keep the constraint
  const int indChairForceY = forceStorage.getStateIndex("seatConstraint_ground_Fy");
  std::vector<double> chairForceYTVec(tVec.size(), 0.0);
  double *chairForceYTVecRawPtr = chairForceYTVec.data();
  forceStorage.getDataColumn(indChairForceY, chairForceYTVecRawPtr);

  std::vector<double> weightVec = expWeightVec(TAU_CHAIR_FORCE, tVec, tVec.back());
  std::transform(weightVec.begin(), weightVec.end(), dtVec.begin(), weightVec.begin(), std::multiplies<double>());
  const double costChairForce = fabs(std::inner_product(chairForceYTVec.begin(), chairForceYTVec.end(), weightVec.begin(), 0.0,
                                                  std::plus<double>(), [](const double chairForce, const double w){
                                                    return w*fabs(chairForce);
                                                  }));

  return costChairForce;
}

std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double T){
  std::vector<double> result(timeVec.size());
  std::transform(timeVec.begin(), timeVec.end(), result.begin(), 
                [tau, T](const double &t){
                  return getExpWeight(tau, t, T);
                });
  return result;
}

#endif