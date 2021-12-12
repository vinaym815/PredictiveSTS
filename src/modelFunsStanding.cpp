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
  const SimTK::Vec2 chairCosts= computeCostsChair(forceStorage);

  const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
  const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);

  const double d0 = eucledianDis(comT0, comTarget);
  const double df = eucledianDis(comTf, comTarget);

  const double progress = 1.0-std::min(d0,df)/d0;
  std::cout << "progress " << progress << std::endl;
  std::cout << "slip Cost " << feetCosts[2] << ", " << chairCosts[1] << std::endl << std::endl; 

  costs[0] = df/d0;
  costs[1] = progress*computeCostJointVel(osimModel, siF);
  costs[2] = progress*feetCosts[3];
  costs[3] = (1.0-progress)*chairCosts[0];
  costs[4] = computeCostActivation(activationTimeSeries);
  costs[5] = computeCostDiffActivation(activationTimeSeries);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = progress*feetCosts[0];
  costs[8] = progress*feetCosts[1];
  costs[9] = progress*(feetCosts[2] + chairCosts[1]);
  #ifdef Assisted
    costs[10] = computeCostAssistance(forceStorage);
  #endif

  //// Clearing the reporters
  muscleActivReporter->clearTable();
};

//// Can not convert to timeseries as it will break the storage due to constraint being disabled
SimTK::Vec2 computeCostsChair(const OpenSim::Storage &forceStorage){
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());
  const std::vector<double> dtVec = dVector(tVec);

  //// Force applied by the ground to keep the constraint
  const int indChairForceX = forceStorage.getStateIndex("seatConstraint_ground_Fx");
  std::vector<double> chairForceXTVec(tVec.size(), 0.0);
  double *chairForceXTVecRawPtr = chairForceXTVec.data();
  forceStorage.getDataColumn(indChairForceX, chairForceXTVecRawPtr);

  const int indChairForceY = forceStorage.getStateIndex("seatConstraint_ground_Fy");
  std::vector<double> chairForceYTVec(tVec.size(), 0.0);
  double *chairForceYTVecRawPtr = chairForceYTVec.data();
  forceStorage.getDataColumn(indChairForceY, chairForceYTVecRawPtr);

  std::vector<double> slipVec(chairForceXTVec.size(), 0.0);
  std::transform(chairForceXTVec.begin(), chairForceXTVec.end(), chairForceYTVec.begin(), slipVec.begin(), 
                  [](const double forceXt, const double forceYt){
                      return std::max(0.0, fabs(forceXt) - mu_static*forceYt);
                  });
  const double slipPenalty =  *std::max_element(slipVec.begin(), slipVec.end());

  double costChairForce = fabs(std::inner_product(chairForceYTVec.begin(), chairForceYTVec.end(), dtVec.begin(), 0.0));
  costChairForce = costChairForce/tVec[tVec.size()-1];

  return SimTK::Vec2{costChairForce, slipPenalty};
}

double computeCostJointVel(const OpenSim::Model &osimModel, const SimTK::State &siF){
  const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
  const double hipVel = coordSet.get("hip_flexion").getSpeedValue(siF);
  const double kneeVel= coordSet.get("knee_angle").getSpeedValue(siF);
  const double ankleVel= coordSet.get("ankle_angle").getSpeedValue(siF);

  return SimTK::convertRadiansToDegrees(fabs(hipVel)+fabs(kneeVel)+fabs(ankleVel));
}

#endif