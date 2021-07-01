// Used to replay the optimization results

#include "OpenSim/OpenSim.h"
#include "parameterization.h"
#include "modelFuns.h"
#include "cmaes.h"
#include "universalConsts.h"
#include "assistanceForceVisualization.h"
#include "termination.h"

#include <iostream>
#include <string>
#include <memory>
#include <algorithm>
#include <sys/stat.h>

const int numComps = 17;
const SimTK::Vec1 lb(-0.2);
const SimTK::Vec1 ub(1.0);
const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;

int main(int argc, char *argv[]){

  // Checking the number of inputs;
  if (!(argc==6)){
      std::cout << "Inappropriate Number of Arguments" << std::endl;
      std::cout << "Correct Format:\n>>funcName ModelName fileName startGeneration endGeneration saveResults(true/false)" << std::endl;
      exit(1);
  }

  // Number of variables per component of muscle excitation parameterization
  int numVarsPerComp = mapNumVarsPerComp[parameterization];
  if (numVarsPerComp != lb.size()){
    std::cout << "Either the lower or upper bound vector was of incorrect size" << std::endl;
    exit(1);
  }

  // Model Name
  const char* modelNameInput = argv[1];

  // Name of the file containing excitation profile
  const char *fileName = argv[2];

  // Generation Numbers
  int genStart = std::stoi(argv[3]);
  int genEnd = std::stoi(argv[4]);

  // Save Results Flag
  bool resultsFlag = false;
  if ((std::string)argv[5] == "true"){
    resultsFlag = true;
  }

  // Adding the dir with geometry files
  OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../../Geometry");
  OpenSim::Model osimModel(modelNameInput);

  //// Setting up the visualization 
  osimModel.setUseVisualizer(true);

  //// Adding the controller to the model
  addController(osimModel, simulationDuration);

  //// For computing costs
  addReporters(osimModel);

  //// Adding the force reporter
  auto frcReporter{std::make_unique<OpenSim::ForceReporter>()};
  frcReporter->setName("forceReporter");
  frcReporter->includeConstraintForces(true);
  osimModel.addAnalysis(frcReporter.get());
  
  //// Adding the assistance force visualization
  #ifdef Assisted
    OpenSim::AssistanceForceVisualization *forceVisualizer = new OpenSim::AssistanceForceVisualization();
    osimModel.addComponent(forceVisualizer);
  #endif

  //// Finalizing the connections
  osimModel.finalizeConnections();
  osimModel.buildSystem();

  //// Adding the standing posture termination
  TerminateSimulation *angleTermination = new TerminateSimulation(osimModel);
  osimModel.updMultibodySystem().addEventHandler(angleTermination);

  //// Adding the seat release event handler
  ReleaseSeatConstraint *releaseSeatConstraint = new ReleaseSeatConstraint(osimModel, SimTK::ConstraintIndex(6), 0.0);
  osimModel.updMultibodySystem().addEventHandler(releaseSeatConstraint); 

  //// Setting up the initial state
  SimTK::State si = osimModel.initializeState();
  si.setTime(initialTime);
  const SimTK::State si0(si);

  SimTK::Visualizer &visualizer = osimModel.updVisualizer().updSimbodyVisualizer();
  visualizer.setShowSimTime(true);

  const int numDecisionVars = numVarsPerComp*numComps*numExtFuncs;
  std::vector<double> controls(numDecisionVars);
  std::vector<double> lowerBoundVec(numDecisionVars), upperBoundVec(numDecisionVars);

  //// Setting up the boundary tranformation
  for(size_t i=0; i<numExtFuncs; ++i){
    for(size_t j=0; j<numComps; ++j){
      for(size_t k=0; k<numVarsPerComp; ++k){
        lowerBoundVec[(numVarsPerComp*i+k)*numComps+j] = lb[k];
        upperBoundVec[(numVarsPerComp*i+k)*numComps+j] = ub[k];
      }
    }
  }
  libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(&lowerBoundVec[0], &upperBoundVec[0], lowerBoundVec.size());
  
  //// Running the simulations
  for(size_t i=0; i<genEnd-genStart+1; ++i){

    ////// Reading up the excitation
    readVector(controls, fileName, i+genStart, i+genStart, 2);
    //readVector(controls, fileName);

    Eigen::VectorXd in = Eigen::Map<Eigen::VectorXd>(controls.data(), controls.size());
    Eigen::VectorXd out = gp.geno(in);

    //// Resetting the initial state    
    si = si0;

    //// Setting up the muscle excitation signals
    setExcitations(osimModel, si, parameterization, numComps, out.data());
    osimModel.equilibrateMuscles(si);

    OpenSim::Manager manager(osimModel);
    std::cout << "WARNING :: Custom Integrator Setting" << std::endl;
    manager.setIntegratorAccuracy(integratorAccuracy);
    manager.setPerformAnalyses(true);

    //// Running the forward simulation
    manager.initialize(si);
    si = manager.integrate(simulationDuration);

    // Saving the .sto files
    if (resultsFlag){
      std::string outputFileName = std::to_string(genStart+i);
      OpenSim::STOFileAdapter_<double>::write(manager.getStatesTable(), outputFileName+".sto");

      #ifdef Standing
        const OpenSim::TableReporterVec3 *comReporter = dynamic_cast<const OpenSim::TableReporterVec3*>(
                                                        &osimModel.getComponent("/comReporter"));
        OpenSim::STOFileAdapter_<SimTK::Vec3>::write(comReporter->getTable(), outputFileName+"_com.mot");
      #endif

      OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>
                                                    (&osimModel.updComponent("/feetForceReporter"));
      OpenSim::STOFileAdapter_<SimTK::SpatialVec>::write(feetForceReporter->getTable(), outputFileName + "_feetForces.mot");

      frcReporter->getForceStorage().print(outputFileName + "_force.mot");
    }

    // Computing the costs
    std::vector<double> costs(numWeights);
    #ifdef Standing
      computeCostsStanding(costs, osimModel, si);
    #else
      computeCostsSitting(osimModel, si0, costs);
    #endif

    const std::vector<double> costWeights{800, 600, 100, 20, 0.4, 10, 20, 0.1, 0.1, 0.2};
    //const std::vector<double> costWeights{5, 4, 200, 40, 10, 20, 1.0, 1.0, 1.0};
    std::vector<double> relativeCosts(costWeights.size());
    std::transform(costWeights.begin(), costWeights.end(), costs.begin(), relativeCosts.begin(),std::multiplies<double>());

    const double result = std::accumulate(relativeCosts.begin(), relativeCosts.end(), 0.0);
    std::cout << "Total Cost : " << result << std::endl;
    for(int i=0; i<costs.size(); ++i){
      std::cout << costs[i] << " : " << relativeCosts[i] << std::endl;
    }

    std::cout << "Sleeping for 2 seconds" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  // Bye bye
  std::cout << "Simulation Completed !" << std::endl;

  return 0;
}