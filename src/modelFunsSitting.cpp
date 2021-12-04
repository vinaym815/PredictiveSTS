#include "modelFuns.h"

#ifndef Standing
void computeCostsSitting(OpenSim::Model &osimModel, const SimTK::State si0, std::vector<double> &costs){

  // Getting pointers to different reporters 
  OpenSim::TableReporter *coordReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/coordReporter"));
  OpenSim::TableReporter *muscleActivReporter = dynamic_cast<OpenSim::TableReporter*>(
                                                    &osimModel.updComponent("/muscleActivReporter"));
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>
                                                    (&osimModel.updComponent("/feetForceReporter"));
  OpenSim::ForceReporter *frcReporter = dynamic_cast<OpenSim::ForceReporter*>
                                                    (&osimModel.updAnalysisSet().get("forceReporter"));

  osimModel.realizePosition(si0);
  const OpenSim::PhysicalOffsetFrame *heelCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame*>
                                                      (&osimModel.updComponent("/bodyset/calcn_r/heel_cnctFrame"));
  const OpenSim::PhysicalOffsetFrame *toesCnctFrame = dynamic_cast<const OpenSim::PhysicalOffsetFrame *>
                                                      (&osimModel.getComponent("/bodyset/toes_r/toes_cnctFrame"));

  const SimTK::Vec3 heelPos = heelCnctFrame->getPositionInGround(si0);                                                    
  const SimTK::Vec3 toesPos = toesCnctFrame->getPositionInGround(si0);                                                    

  const auto &activationTimeSeries = muscleActivReporter->getTable();
  const auto &coordTimeSeries = coordReporter->getTable();
  const auto &feetWrenchTimesSeries = feetForceReporter->getTable();

  const auto &forceStorage = frcReporter->getForceStorage();

  const SimTK::Vec2 costsCoordinate = computeCostsCoordinate(coordTimeSeries);
  const SimTK::Vec3 feetCosts = computeCostFeet(feetWrenchTimesSeries, heelPos, toesPos);

  // Filling up the cost matrix
  costs[0] = costsCoordinate[0];
  costs[1] = costsCoordinate[1];
  costs[2] = computeCostActivation(activationTimeSeries, simulationDuration);
  costs[3] = computeCostDiffActivation(activationTimeSeries, simulationDuration);
  costs[4] = computeCostLimitTorque(forceStorage);
  costs[5] = feetCosts[0];
  costs[6] = feetCosts[1];
  costs[7] = feetCosts[2];

  #ifdef Assisted
    costs[8] = computeCostAssistance(forceStorage);
  #endif

  // Clearing all the reporters
  muscleActivReporter->clearTable();
  coordReporter->clearTable();
  feetForceReporter->clearTable();
};

SimTK::Vec2 computeCostsCoordinate(const OpenSim::TimeSeriesTable &coordTimeSeries){
  double cost_coordinate_value{0}, cost_coordinate_speed{0};
  const int numRelevantCoordinate = coordTimeSeries.getNumColumns()/2;

  for(size_t iCoord = 0; iCoord<numRelevantCoordinate; ++iCoord){
    auto coordValueVec = coordTimeSeries.getDependentColumnAtIndex(2*iCoord);
    auto coordSpeedVec = coordTimeSeries.getDependentColumnAtIndex(2*iCoord+1);
    const double valueT0 = coordValueVec[0];
    const double speedT0 = coordSpeedVec(0);

    cost_coordinate_value += std::accumulate(coordValueVec.begin(), coordValueVec.end(), 0.0,
                              [&valueT0](const double &lhs, const double &rhs){
                                return lhs+((rhs-valueT0)*(rhs-valueT0));
                              });
    cost_coordinate_speed += std::accumulate(coordSpeedVec.begin(), coordSpeedVec.end(), 0.0,
                              [&speedT0](const double &lhs, const double &rhs){
                                return lhs+((rhs-speedT0)*(rhs-speedT0));
                              });
  }
  const int coordNumElements = numRelevantCoordinate*coordTimeSeries.getNumRows();

  // rms value
  cost_coordinate_value = SimTK::convertRadiansToDegrees(sqrt(cost_coordinate_value/coordNumElements));
  cost_coordinate_speed = SimTK::convertRadiansToDegrees(sqrt(cost_coordinate_speed/coordNumElements));

  return SimTK::Vec2{cost_coordinate_value, cost_coordinate_speed};
};

#endif