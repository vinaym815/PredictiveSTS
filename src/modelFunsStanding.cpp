#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State &si0, const SimTK::State &siF, 
                          const double seatReleaseTime){

  OpenSim::TableReporterVec3 *comReporter = dynamic_cast<OpenSim::TableReporterVec3*>(
                                                    &osimModel.updComponent("comReporter"));
  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>(
                                                    &osimModel.updComponent("/feetForceReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>(
                                                    &osimModel.updAnalysisSet().get("forceReporter"));

  osimModel.realizePosition(si0);
  const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                      (&osimModel.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
  const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame *>
                                                      (&osimModel.getComponent("/bodyset/toes_r/toes_cnctFrame"));

  //const OpenSim::Body &talus = osimModel.getBodySet().get("talus_r");
  //const SimTK::Vec3 feetPos = talus.getPositionInGround(si0);
  const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);                                                    
  const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);                                                    

  const auto &comTimeSeries = comReporter->getTable();
  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &feetWrenchTimesSeries = feetForceReporter->getTable();

  //// Can not convert to timeseries as it will break the storage due to constraint being disabled
  const auto &forceStorage = frcReporter->getForceStorage(); 
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> forceTimeVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());

  const std::vector<double> &reporterTimeVec = comReporter->getTable().getIndependentColumn();
  const double tau_ChairForce = simulationDuration*tau_ChairForce_pct;
  const double tau_Boundary = simulationDuration*tau_Boundary_pct;

  std::vector<double> wTauBoundaryVec = expWeightVec(tau_Boundary, reporterTimeVec, simulationDuration);
  std::vector<double> wTauChairForceVec = expWeightVec(tau_ChairForce, forceTimeVec, simulationDuration);
  std::vector<double> wTauChairEqSpacing = expWeightVec(tau_ChairForce, reporterTimeVec, simulationDuration);

  const SimTK::Vec3 feetCosts = computeCostFeet(feetWrenchTimesSeries, heelPos, toesPos);
  const SimTK::Vec2 chairCosts= computeCostsChair(wTauChairForceVec, forceStorage);

  const double bodyWeight = osimModel.getTotalMass(si0)*(osimModel.getGravity()[1]);
  const SimTK::Vec3 comT0 = osimModel.calcMassCenterPosition(si0);
  const SimTK::Vec3 comTf = osimModel.calcMassCenterPosition(siF);
  const SimTK::Vec3 comVelTf = osimModel.calcMassCenterVelocity(siF);

  const double d0 = eucledianDis(comT0, comTarget);
  const double df = eucledianDis(comTf, comTarget);

  const double progress = 1.0-std::min(d0,df)/d0;
  std::cout << "progress " << progress << std::endl;

  costs[0] = df/d0;
  costs[1] = progress*eucledianDis(comVelTf, SimTK::Vec3 {0.0});
  costs[2] = progress*computeCostFeetForce(wTauChairEqSpacing, feetWrenchTimesSeries, bodyWeight);
  costs[3] = (1.0-progress)*chairCosts[0];
  costs[4] = computeCostActivation(activationTimeSeries, seatReleaseTime);
  costs[5] = computeCostDiffActivation(activationTimeSeries, seatReleaseTime);
  costs[6] = computeCostLimitTorque(forceStorage);
  costs[7] = feetCosts[0];
  costs[8] = feetCosts[1];
  costs[9] = feetCosts[2] + chairCosts[1];
  #ifdef Assisted
    costs[10] = computeCostAssistance(forceStorage);
  #endif

  //// Clearing the reporters
  comReporter->clearTable();
  muscleActivReporter->clearTable();
  feetForceReporter->clearTable();
};

double bestEucledianDisToTarget(const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);

  std::vector<double> disVec(comTPosVec.nrow());
  std::transform(comTPosVec.begin(), comTPosVec.end(), disVec.begin(),[](const SimTK::Vec3 &comT){
    return eucledianDis(comT, comTarget);
  });

  double result = *std::min_element(disVec.begin(),disVec.end());

  //// Returning Normalized Best Distance
  return result/disVec[0];
}

double computeCostFeetForce(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, 
                            const double bodyWeight){
  const int indfeetWrench = feetWrenchTimeSeries.getColumnIndex("/jointset/ground_calcn_r|reaction_on_parent");
  auto feetWrenchVec = feetWrenchTimeSeries.getDependentColumnAtIndex(indfeetWrench);
  double result = 0.0;
  for(int i=0; i<weightVec.size(); ++i){
    result += weightVec[i]*fabs(feetWrenchVec[i][1][1] - bodyWeight);
  }
  result *= reportInterval;
  return result;
}

SimTK::Vec2 computeCostsChair(const std::vector<double> &weightVec, const OpenSim::Storage &forceStorage){
  const int indChairForceX = forceStorage.getStateIndex("seatConstraint_ground_Fx");
  std::vector<double> chairForceXTVec(weightVec.size(), 0.0);
  double *chairForceXTVecRawPtr = chairForceXTVec.data();
  forceStorage.getDataColumn(indChairForceX, chairForceXTVecRawPtr);

  const int indChairForceY = forceStorage.getStateIndex("seatConstraint_ground_Fy");
  std::vector<double> chairForceYTVec(weightVec.size(), 0.0);
  double *chairForceYTVecRawPtr = chairForceYTVec.data();
  forceStorage.getDataColumn(indChairForceY, chairForceYTVecRawPtr);

  std::vector<double> tVec(weightVec.size(), 0.0);
  double *tVecStart = tVec.data();
  forceStorage.getTimeColumn(tVecStart);
  const std::vector<double> dtVec = dVector(tVec);

  std::vector<double> frictionPenaltyVec(chairForceXTVec.size(), 0.0);
  std::transform(chairForceXTVec.begin(), chairForceXTVec.end(), chairForceYTVec.begin(), frictionPenaltyVec.begin(), 
                  [](const double forceXt, const double forceYt){
                      return bool(fabs(forceXt) > mu_static*forceYt)*(fabs(forceXt) - mu_static*forceYt);
                  });
  const double slipPenalty =  std::inner_product(frictionPenaltyVec.begin(), frictionPenaltyVec.end(), dtVec.begin(), 0.0);

  std::vector<double> chairForceYPenaltyVec(dtVec.size(), 0.0);
  std::transform( chairForceYTVec.begin(), chairForceYTVec.end(), dtVec.begin(), chairForceYPenaltyVec.begin(), std::multiplies<double>());

  const double costChairForce = fabs(std::inner_product(chairForceYPenaltyVec.begin(), chairForceYPenaltyVec.end(), weightVec.begin(), 0.0));
  return SimTK::Vec2{costChairForce, slipPenalty};
}

std::vector<double> expWeightVec(const double tau, const std::vector<double> &timeVec, const double tF){
  std::vector<double> result(timeVec.size());
  std::transform(timeVec.begin(), timeVec.end(), result.begin(), 
                [tau, tF](const double &t){
                  return getIncExpWeight(tau, t, tF);
                });
  return result;
}

#endif