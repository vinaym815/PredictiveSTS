#include "modelFuns.h"

#ifdef Standing
/*
Computes the cost for standing simulation
*/
void computeCostsStanding(std::vector<double> &costs, OpenSim::Model &osimModel, const SimTK::State si0, 
                          const double seatReleaseTime, const double tF){

  OpenSim::TableReporterVec3 *comReporter = dynamic_cast<OpenSim::TableReporterVec3*>(
                                                    &osimModel.updComponent("comReporter"));
  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>(
                                                    &osimModel.updComponent("/feetForceReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>(
                                                    &osimModel.updAnalysisSet().get("forceReporter"));
  OpenSim::TableReporter *coordReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/coordReporter"));

  osimModel.realizePosition(si0);
  const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                      (&osimModel.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
  const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame *>
                                                      (&osimModel.getComponent("/bodyset/toes_r/toes_cnctFrame"));

  const OpenSim::Body &talus = osimModel.getBodySet().get("talus_r");
  const SimTK::Vec3 feetPos = talus.getPositionInGround(si0);
  const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);                                                    
  const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);                                                    

  const auto &comTimeSeries = comReporter->getTable();
  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &feetWrenchTimesSeries = feetForceReporter->getTable();
  const auto &coordTimeSeries = coordReporter->getTable();

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

  const double progress = computeComProgess(comTimeSeries);
  const double vin = 1.0/(1+std::pow(SimTK::E, -(4-8*progress)));

  std::cout << "progress " << 1-progress << std::endl;
  std::cout << "vin " << vin << std::endl;

  const double bodyWeight = osimModel.getTotalMass(si0)*(osimModel.getGravity()[1]);
  //// Filling up the cost matrix
  //if(tF<simulationDuration-1e-2){
  //  costs[0] = -1.25;
  //}
  //else{
  //}

  costs[0] = computeCostComY(wTauBoundaryVec, comTimeSeries);
  costs[1] = computeCostComX(wTauBoundaryVec, comTimeSeries, feetPos);
  costs[2] = vin*computeCostFeetForce(wTauChairEqSpacing, feetWrenchTimesSeries, bodyWeight);
  costs[3] = (1.0-vin)*chairCosts[0];
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
  coordReporter->clearTable();
};

double computeComProgess(const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  double result = 10.0;
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);

  const SimTK::Vec3 &comT0 = comTPosVec[0];
  const double distance0 = std::sqrt(std::pow(comT0[0]-comTarget[0], 2) + std::pow(comT0[1]-comTarget[1], 2));

  for(int i=0; i<comTPosVec.nrow(); ++i){
    const SimTK::Vec3 &comT = comTPosVec[i];
    const double distanceT = std::sqrt(std::pow(comT[0]-comTarget[0],2) + 
                                      std::pow(comT[1]-comTarget[1],2));
    if(distanceT<result){
      result = distanceT;
    }
  }
  return result/distance0;
}

double computeCostComY(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  const SimTK::Vec3 comT0 = comTPosVec[0];

  double result = std::inner_product(weightVec.begin(), weightVec.end(), comTPosVec.begin(), 0.0, std::plus<double>(),
                                          [&comT0](const double &w, const SimTK::Vec3 &comT){
                                            return (comT0[1] - comT[1]);
                                          });
  result *= reportInterval;
  return result;
}

double computeCostComX(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries, 
                        const SimTK::Vec3 feetPos){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_position");
  auto comTPosVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  const SimTK::Vec3 comT0 = comTPosVec[0];
  double result = std::inner_product(weightVec.begin(), weightVec.end(), comTPosVec.begin(), 0.0, std::plus<double>(),
                                          [&comT0, &feetPos](const double &w, const SimTK::Vec3 &comT){
                                            return fabs(feetPos[0] - comT[0]);
                                          });
  result *= reportInterval;                
  return result;
}

double computeBestPosture(const OpenSim::TimeSeriesTable coordTimeSeries){
  std::vector<double> result(coordTimeSeries.getNumRows());
  const size_t hipInd = coordTimeSeries.getColumnIndex("/jointset/hip/hip_flexion|value");
  const size_t kneeInd = coordTimeSeries.getColumnIndex("/jointset/walker_knee/knee_angle|value");
  const size_t ankleInd = coordTimeSeries.getColumnIndex("/jointset/ankle/ankle_angle|value");
  const SimTK::MatrixView &coordMat = coordTimeSeries.getMatrix();

  for(size_t i=0; i<coordMat.nrow(); ++i){
    const double hipAngle = coordMat.getAnyElt(i, hipInd);
    const double kneeAngle = coordMat.getAnyElt(i, kneeInd);
    const double ankleAngle = coordMat.getAnyElt(i, ankleInd);

    const double tibiaAngleVertical = ankleAngle;
    const double femurAngleVertical = ankleAngle-kneeAngle; 
    const double torsoAngleVertical = ankleAngle-kneeAngle+hipAngle;

    result[i] = fabs(tibiaAngleVertical) + fabs(femurAngleVertical) + fabs(torsoAngleVertical);
  }
  const double answer = SimTK::convertRadiansToDegrees(*std::min_element(result.begin(), result.end()));
  return answer;
}

double computeCostCoordinate(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable coordTimeSeries){
  double result = 0.0;
  const size_t hipInd = coordTimeSeries.getColumnIndex("/jointset/hip/hip_flexion|value");
  const size_t kneeInd = coordTimeSeries.getColumnIndex("/jointset/walker_knee/knee_angle|value");
  const size_t ankleInd = coordTimeSeries.getColumnIndex("/jointset/ankle/ankle_angle|value");
  const SimTK::MatrixView &coordMat = coordTimeSeries.getMatrix();

  for(size_t i=0; i<coordMat.nrow(); ++i){
    const double hipAngle = coordMat.getAnyElt(i, hipInd);
    const double kneeAngle = coordMat.getAnyElt(i, kneeInd);
    const double ankleAngle = coordMat.getAnyElt(i, ankleInd);

    const double tibiaAngleVertical = ankleAngle;
    const double femurAngleVertical = ankleAngle-kneeAngle; 
    const double torsoAngleVertical = ankleAngle-kneeAngle+hipAngle;

    result += weightVec[i]*(fabs(tibiaAngleVertical) + fabs(femurAngleVertical) + fabs(torsoAngleVertical));
  }
  result = SimTK::convertRadiansToDegrees(result)*reportInterval;
  return result;
}

double computeCostCoordinateVel(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTable coordTimeSeries){
  double result = 0.0;
  const size_t hipVelInd = coordTimeSeries.getColumnIndex("/jointset/hip/hip_flexion|speed");
  const size_t kneeVelInd = coordTimeSeries.getColumnIndex("/jointset/walker_knee/knee_angle|speed");
  const size_t ankleVelInd = coordTimeSeries.getColumnIndex("/jointset/ankle/ankle_angle|speed");
  const SimTK::MatrixView &coordMat = coordTimeSeries.getMatrix();

  for(size_t i=0; i<coordMat.nrow(); ++i){
    const double hipVel = coordMat.getAnyElt(i, hipVelInd);
    const double kneeVel = coordMat.getAnyElt(i, kneeVelInd);
    const double ankleVel = coordMat.getAnyElt(i, ankleVelInd);

    const double tibiaWGround = ankleVel;
    const double femurWGround = ankleVel-kneeVel; 
    const double torsoWGround = ankleVel-kneeVel+hipVel;
    result += weightVec[i]*(fabs(tibiaWGround) + fabs(femurWGround) + fabs(torsoWGround));
  }
  result = SimTK::convertRadiansToDegrees(result)*reportInterval;
  return result;
}

double computeCostComVel(const std::vector<double> &weightVec, const OpenSim::TimeSeriesTableVec3 &comTimeSeries){
  const size_t indCom = comTimeSeries.getColumnIndex("/|com_velocity");
  auto comVelVec = comTimeSeries.getDependentColumnAtIndex(indCom);
  double result = std::inner_product(weightVec.begin(), weightVec.end(), comVelVec.begin(), 0.0, std::plus<double>(),
                                          [](const double &w, const SimTK::Vec3 &comVelT){
                                            return (w*std::sqrt(comVelT[0]*comVelT[0] + comVelT[1]*comVelT[1]));
                                          });

  result *= reportInterval;
  return result;
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