#include "modelFuns.h"

// Adds a prescribed controller with piecewise linear functions to the model and saves a copy of it 
void addComponentsToModel(const std::string modelName, const std::string newModelName, const double tUB){
    OpenSim::Model osimModel(modelName);
    addController(osimModel, tUB);
    osimModel.finalizeConnections();
    osimModel.print(newModelName);
};

void addReporters(OpenSim::Model &osimModel){
  //// Adding reporter for the forces applied on the ground by the feet
  OpenSim::Joint &groundFeetJoint = osimModel.updJointSet().get("ground_calcn_r");
  OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = new OpenSim::TableReporter_<SimTK::SpatialVec>();
  feetForceReporter->set_report_time_interval(reportInterval);
  feetForceReporter->addToReport(groundFeetJoint.getOutput("reaction_on_parent"));
  feetForceReporter->setName("feetForceReporter");
  osimModel.addComponent(feetForceReporter);

  //// Adding the activation reporter
  OpenSim::TableReporter *muscleActivReporter = new OpenSim::TableReporter();
  muscleActivReporter->set_report_time_interval(reportInterval);
  muscleActivReporter->setName("muscleActivReporter");
  OpenSim::Set<OpenSim::Muscle> &muscleSet = osimModel.updMuscles();
  for(size_t i=0; i<muscleSet.getSize(); ++i){
    muscleActivReporter->addToReport(muscleSet[i].getOutput("activation"));
  }

  //// Adding activation from the first order dynamics of the external assistance
  #ifdef Assisted
    OpenSim::ActivationPointActuator *assistFx = dynamic_cast<OpenSim::ActivationPointActuator *>(&osimModel.updForceSet().get("assistFx"));
    OpenSim::ActivationPointActuator *assistFy = dynamic_cast<OpenSim::ActivationPointActuator *>(&osimModel.updForceSet().get("assistFy"));
    muscleActivReporter->addToReport(assistFx->getOutput("activation"));
    muscleActivReporter->addToReport(assistFy->getOutput("activation"));
  #endif

  osimModel.addComponent(muscleActivReporter);

  #ifdef Standing
    //// Adding the COM position and velocity reporter
    OpenSim::TableReporterVec3 *comReporter = new OpenSim::TableReporterVec3();
    comReporter->set_report_time_interval(reportInterval);
    comReporter->setName("comReporter");
    comReporter->addToReport(osimModel.getOutput("com_position"));
    comReporter->addToReport(osimModel.getOutput("com_velocity"));
    osimModel.addComponent(comReporter);
  #else
    //// Adding coordiante value and speed reporter
    const OpenSim::CoordinateSet &coordSet = osimModel.getCoordinateSet();
    const std::vector<int> coordinateIndices{coordSet.getIndex("hip_flexion"), 
                                              coordSet.getIndex("knee_angle"), 
                                              coordSet.getIndex("ankle_angle")};
    OpenSim::TableReporter* coordReporter = new OpenSim::TableReporter();
    coordReporter->set_report_time_interval(reportInterval);
    coordReporter->setName("coordReporter");
    for(auto ind : coordinateIndices){
      coordReporter->addToReport(coordSet[ind].getOutput("value"));
      coordReporter->addToReport(coordSet[ind].getOutput("speed"));
    }
    osimModel.addComponent(coordReporter);
  #endif
};

// Adds an open Loop controller for muscles
void addController(OpenSim::Model &osimModel, const double tUB){
  const OpenSim::Set<OpenSim::Actuator> &actuators = osimModel.getActuators();
  OpenSim::PrescribedController *openLoopController = new OpenSim::PrescribedController();
  openLoopController->setName("openLoopController");
  openLoopController->setActuators(actuators);

  /// Default neural excitation function
  const int nSamples = int(tUB/samplingDt)+1;
  SimTK::Vector vecExt(nSamples, defaultExcitationController);
  SimTK::Vector vecTime(nSamples);
  std::iota(vecTime.begin(), vecTime.end(), 0);
  std::for_each(vecTime.begin(), vecTime.end(), [](double &value){ value*=samplingDt;});

  // Setting up the individual neural excitation function for different acutators
  for(size_t i=0; i<actuators.getSize(); i++){
    OpenSim::PiecewiseLinearFunction* func = new OpenSim::PiecewiseLinearFunction(nSamples, 
                                                  vecTime.getContiguousScalarData(), 
                                                  vecExt.getContiguousScalarData(), "ExcitationSignal");
    openLoopController->prescribeControlForActuator( actuators[i].getName(), func);
  }
  osimModel.addController(openLoopController);
};

/*
This function interpolates node values for the controller from the neural excitation parameterizations
Sets u0 = a0
*/
void setExcitations(OpenSim::Model &osimModel, SimTK::State &si0, const ParameterizationType parameterization, 
                                              const int numComps, const double *compValues){ 

  OpenSim::PrescribedController *openLoopController = dynamic_cast<OpenSim::PrescribedController*>(
                                                    &osimModel.updComponent("/controllerset/openLoopController"));
  OpenSim::FunctionSet &actuatorExtFuncs = openLoopController->upd_ControlFunctions();
  const int numActuators = actuatorExtFuncs.getSize();
  const int numVarsPerComp = mapNumVarsPerComp[parameterization];

  OpenSim::Set<OpenSim::Actuator> &actuatorSet = osimModel.updActuators();
  for(size_t i=0; i<numActuators; ++i){
    OpenSim::PiecewiseLinearFunction *actuatorExtFuncPtr = dynamic_cast<OpenSim::PiecewiseLinearFunction*>
                                                                                  (&actuatorExtFuncs[i]);

    const SimTK::Vector params(numVarsPerComp*numComps, &compValues[numVarsPerComp*numComps*i]);
    const std::unique_ptr<CustomFunction> parameterizationExtFuncPtr = mapExtFuncPtr[parameterization](params);

    OpenSim::ScalarActuator *actuator = dynamic_cast<OpenSim::ScalarActuator*>(&actuatorSet[i]);
    if(actuator == NULL){
      std::cout << "Could not convert to a scalar actuator" << std::endl;
    }
    const double lowerControlLimit = actuator->get_min_control();
    const double upperControlLimit = actuator->get_max_control();

    // Setting up the excitation signal
    for(size_t j=0; j<actuatorExtFuncPtr->getNumberOfPoints(); ++j){
      double excitation = parameterizationExtFuncPtr->getValue(actuatorExtFuncPtr->getX(j));

      // Clamping is required for correct bheavior of first order dynamics
      excitation = SimTK::clamp(lowerControlLimit, excitation, upperControlLimit);
      actuatorExtFuncPtr->setY(j, excitation);
    }

    // Setting a0 = u0
    OpenSim::Muscle *muscle = dynamic_cast<OpenSim::Muscle*>(&actuatorSet[i]);
    if(muscle != NULL){
      muscle->setActivation(si0, actuatorExtFuncPtr->getY(0));
    }
  }
};

// Runs a forward simulation and returns the computed costs
std::vector<double> runSimulation(OpenSim::Model &osimModel, const ParameterizationType &parameterization, 
                                                              const double *controls, const int numComps,
                                                              const bool visualizeResults,
                                                              const bool saveResults,
                                                              const std::string outputPrefix){

  //// Adding the reporters
  addReporters(osimModel);
  osimModel.setUseVisualizer(visualizeResults);

  // Adding the force reporter
  auto frcReporter{std::make_unique<OpenSim::ForceReporter>()};
  frcReporter->setName("forceReporter");
  frcReporter->includeConstraintForces(true);
  osimModel.addAnalysis(frcReporter.get());

  // Assistance force visualization  
  #ifdef Assisted
    OpenSim::AssistanceForceVisualization *forceVisualizer = new OpenSim::AssistanceForceVisualization();
    osimModel.addComponent(forceVisualizer);
  #endif

  // Building the model 
  osimModel.finalizeConnections();
  osimModel.buildSystem();

  if(visualizeResults)
    osimModel.updVisualizer().updSimbodyVisualizer().setShowSimTime(true);

  //// Adding the standing posture termination
  TerminateSimulation *angleTermination = new TerminateSimulation(osimModel);
  osimModel.updMultibodySystem().addEventHandler(angleTermination);

  // Adding the seat release constraint
  ReleaseSeatConstraint *releaseSeatConstraint = new ReleaseSeatConstraint(osimModel, SimTK::ConstraintIndex(6), 0.0);
  osimModel.updMultibodySystem().addEventHandler(releaseSeatConstraint); 

  SimTK::State &si = osimModel.initializeState();
  si.setTime(initialTime);
  const SimTK::State si0(si); 

  // Default costs
  std::vector<double> costs(numWeights);
  std::fill(costs.begin(), costs.end(), 1e6);

  try{
    // Setting the excitation
    setExcitations(osimModel, si, parameterization, numComps, controls);
    osimModel.equilibrateMuscles(si);

    // Performing the forward simulation
    OpenSim::Manager manager(osimModel);
    manager.setIntegratorAccuracy(integratorAccuracy);
    manager.initialize(si);

    manager.integrate(simulationDuration);
    const double terminationTime = angleTermination->getTerminationTime();
    const double seatReleaseTime = releaseSeatConstraint->getSeatRleaseTime();

    // The computingCosts function clears the reporter memories. Therefore, exports needs to be done before the cost is computed
    #ifdef Standing
      if(saveResults){
        OpenSim::STOFileAdapter_<double>::write(manager.getStatesTable(), outputPrefix+".sto");
        frcReporter->getForceStorage().print(outputPrefix + "_force.mot");
        OpenSim::TableReporter_<SimTK::SpatialVec> *feetForceReporter = dynamic_cast<OpenSim::TableReporter_<SimTK::SpatialVec>*>
                                                (&osimModel.updComponent("/feetForceReporter"));
        OpenSim::STOFileAdapter_<SimTK::SpatialVec>::write(feetForceReporter->getTable(), outputPrefix + "_feetForces.mot");

        const OpenSim::TableReporterVec3 *comReporter = dynamic_cast<const OpenSim::TableReporterVec3*>(
                                                    &osimModel.getComponent("/comReporter"));
        OpenSim::STOFileAdapter_<SimTK::Vec3>::write(comReporter->getTable(), outputPrefix+"_com.mot");
      }
        computeCostsStanding(costs, osimModel, si0, seatReleaseTime, terminationTime);
    #else
        computeCostsSitting(osimModel, si0, costs);
    #endif

  }
  catch(const std::string& ex){
    std::cout << ex << std::endl;
    std::cout << "Sending Default Costs" << std::endl;
    std::cout << std::endl;
  }
  catch(const std::exception& ex){
    std::cout << ex.what() << std::endl;
    std::cout << "Sending Default Costs" << std::endl;
  }

  return costs;
};

// Computes the penalty for using the mechanical limits at joints
double computeCostLimitTorque(const OpenSim::Storage &forceStorage){
  double result{0};

  OpenSim::Array<double> timeArray;
  forceStorage.getTimeColumn(timeArray);
  const std::vector<double> tVec(timeArray.get(), timeArray.get()+timeArray.size());
  const std::vector<double> dtVec = dVector(tVec);

  const std::vector<std::string> limitTorqueLabels{"hip_flexion_LimitForce", "knee_angle_LimitForce", 
                                              "ankle_angle_LimitForce"};
  for(auto limitTorqueLabel: limitTorqueLabels){
    std::vector<double> limitTorqueVec(dtVec.size(), 0.0);
    double *limitTorqueVecPtr = limitTorqueVec.data();
    forceStorage.getDataColumn(limitTorqueLabel, limitTorqueVecPtr);
    result += std::inner_product(limitTorqueVec.begin(), limitTorqueVec.end(), dtVec.begin(), 0.0, 
                                  std::plus<double>(), [](const double &limitTorqueT, const double &dt){
                                    return fabs(limitTorqueT)*dt;
                                });
  }
  result = result/(limitTorqueLabels.size());
  return result;
}

// Computes the cost associated with muscle activation
double computeCostActivation(const OpenSim::TimeSeriesTable &activationTimeSeries, const double chairContactLossTime){
  double result{0};
  const size_t offset = activationTimeSeries.getNearestRowIndexForTime(chairContactLossTime);
  for(size_t i=0; i<activationTimeSeries.getNumColumns(); ++i){
    SimTK::VectorView activVec = activationTimeSeries.getDependentColumnAtIndex(i);
    result += std::inner_product(activVec.begin(), activVec.end() - offset, activVec.begin(), 0.0);
  }

  // Averaging over all muscles
  result = sqrt(result*reportInterval/activationTimeSeries.getNumColumns());
  return result;
}

// Computes the cost associated with rate of change of muscle activation
double computeCostDiffActivation(const OpenSim::TimeSeriesTable &activationTimeSeries, const double chairContactLossTime){
  double result{0};
  const size_t offset = activationTimeSeries.getNearestRowIndexForTime(chairContactLossTime);
  for(size_t i=0; i<activationTimeSeries.getNumColumns(); ++i){
    SimTK::VectorView activVec = activationTimeSeries.getDependentColumnAtIndex(i);
    const std::vector<double> dActivVec = dVector(activVec);
    result += std::inner_product(dActivVec.begin()+offset, dActivVec.end(), dActivVec.begin()+offset, 0.0);
  }

  // Averaging over all Actuators
  result = sqrt(result/(reportInterval*activationTimeSeries.getNumColumns()));
  return result;
}

// Computes the costs associated with feet
SimTK::Vec3 computeCostFeet(const OpenSim::TimeSeriesTable_<SimTK::SpatialVec> &feetWrenchTimeSeries, const SimTK::Vec3 heelPos, 
                            const SimTK::Vec3 toesPos){
  double costZMP = 0.0;
  double costUnilaterality = 0.0;
  double costSlip = 0.0;

  Eigen::Matrix2d A;
  A << heelPos[0], toesPos[0], 1, 1;
  const Eigen::Matrix2d Ainv = A.inverse();
  const int indfeetWrench = feetWrenchTimeSeries.getColumnIndex("/jointset/ground_calcn_r|reaction_on_parent");
  auto feetWrenchVec = feetWrenchTimeSeries.getDependentColumnAtIndex(indfeetWrench);
  Eigen::Vector2d b;
  for(int i=0; i<feetWrenchVec.nrow(); ++i){
    const SimTK::SpatialVec &feetWrenchT = feetWrenchVec[i];
    const double torqueZt = feetWrenchT[0][2];
    const double forceXt = feetWrenchT[1][0];
    const double forceYt = feetWrenchT[1][1];
    const double zmp = torqueZt/forceYt;

    b << torqueZt, forceYt;  
    const Eigen::Vector2d heelToeForces = Ainv*b;

    costZMP += fabs(zmp - (heelPos[0]+toesPos[0])/2);
    costUnilaterality += bool(heelToeForces[0]>0)*heelToeForces[0] + bool(heelToeForces[1]>0)*heelToeForces[1];
    costSlip += bool(fabs(forceXt) > mu_static*fabs(forceYt))*(fabs(forceXt) - mu_static*fabs(forceYt));
  }

  costZMP *= reportInterval;
  costUnilaterality *= reportInterval;
  costSlip *= reportInterval;

  return SimTK::Vec3(costZMP, costUnilaterality, costSlip);
}

#ifdef Assisted
// Computes the cost associated with external assistance
double computeCostAssistance(const OpenSim::Storage &forceStorage){
  OpenSim::Array<double> forceTimeArray;
  forceStorage.getTimeColumn(forceTimeArray);
  const std::vector<double> tVec(forceTimeArray.get(), forceTimeArray.get()+forceTimeArray.getSize());
  const std::vector<double> dtVec = dVector(tVec);

  const int indAssistFx = forceStorage.getStateIndex("assistFx");
  const int indAssistFy = forceStorage.getStateIndex("assistFy");

  std::vector<double> assistFxTVec(tVec.size(), 0.0);
  double *assistFxTVecRawPtr = assistFxTVec.data();
  forceStorage.getDataColumn(indAssistFx, assistFxTVecRawPtr);

  std::vector<double> assistFyTVec(tVec.size(), 0.0);
  double *assistFyTVecRawPtr = assistFyTVec.data();
  forceStorage.getDataColumn(indAssistFy, assistFyTVecRawPtr);

  std::vector<double> assistForceTVec(tVec.size(), 0.0);
  std::transform(assistFxTVec.begin(), assistFxTVec.end(), assistFyTVec.begin(), assistForceTVec.begin(), 
                 [](const double fX, const double fY){
                   return std::sqrt(fX*fX + fY*fY);
                 });

  const double result = std::inner_product(assistForceTVec.begin(), assistForceTVec.end(), dtVec.begin(), 0.0);
  return result;
}
#endif

// Saves the log and resume files
void progressFunc(const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols, const std::string logFolder){
    using namespace libcmaes;
    
    // Writing resume mean file
    Eigen::VectorXd xMean = cmasols.xmean();
    std::vector<double> xMeanVector(xMean.data(), xMean.data() + xMean.rows() * xMean.cols());
    writeVector(xMeanVector, logFolder+resumeMeanFileName , std::ios::out);
    
    // Writing resume covariance file
    Eigen::MatrixXd covMat = cmasols.cov();
    Eigen::Map<Eigen::VectorXd> covMatFlat(covMat.data(), covMat.size());
    std::vector<double> covVec(covMatFlat.data(), covMatFlat.data()+covMatFlat.rows()*covMatFlat.cols());
    writeVector(covVec, logFolder+resumeCovFileName, std::ios::out);

    // Writing the sigma
    std::vector<double> sigmaVec(1);
    sigmaVec[0] = cmasols.sigma();
    writeVector(sigmaVec, logFolder+resumeSigmaFileName, std::ios::out);
    
    // Writing the best candidate for the generation
    std::vector<double> vecOutBest;
    vecOutBest.push_back(cmasols.niter());
    vecOutBest.push_back(cmasols.best_candidate().get_fvalue());
    Eigen::VectorXd bestparameters = cmaparams.get_gp().pheno(cmasols.best_candidate().get_x_dvec());
    vecOutBest.insert(vecOutBest.end(), bestparameters.data(), bestparameters.data() + bestparameters.rows() * bestparameters.cols());
    writeVector(vecOutBest, logFolder+logBestFileName);
};

// Loads the mean, stdDev and sigma from files
CMASolutions resumeDistribution(const std::string dirName, CMAParameters<GenoPhenoType> &cmaparams){

    // Launching message
    std::cout << "Resuming Distribution" << std::endl;

    std::vector<double> resumeMeanVec(cmaparams.dim());
    readVector(resumeMeanVec, dirName+resumeMeanFileName);
    Eigen::Map<Eigen::VectorXd> resumeXMean(resumeMeanVec.data(), resumeMeanVec.size());

    std::vector<double> resumeCovVec(cmaparams.dim()*cmaparams.dim());
    readVector(resumeCovVec, dirName+resumeCovFileName);
    Eigen::MatrixXd resumeCovMat = Eigen::Map<Eigen::MatrixXd>(resumeCovVec.data(), cmaparams.dim(), cmaparams.dim());

    std::vector<double> resumeSigmaVec(1);
    readVector(resumeSigmaVec, dirName+resumeSigmaFileName);
    double sigma = resumeSigmaVec[0];

    CMASolutions resumeSolution(cmaparams);
    resumeSolution.set_xmean(resumeXMean);
    resumeSolution.set_sigma(sigma);
    resumeSolution.set_cov(resumeCovMat);

    return resumeSolution;
}

std::string createUniqueFolder(const std::string path){
  const std::chrono::seconds timeStamp = std::chrono::duration_cast<std::chrono::seconds>(
      std::chrono::system_clock::now().time_since_epoch());
  const std::string logFolder = path + std::to_string(timeStamp.count())+"/";
  mkdir(path.c_str(), 0777);
  mkdir(logFolder.c_str(), 0777);
  return logFolder;
};

// Used to read a set of lines from a file
void readVector(std::vector<double> &vecOutput, std::string fileName, const int startLine, const int endLine, const int offset){

  // Opening up the file
  std::fstream fin;
  fin.open(fileName, std::ios::in);
  std::string line, value;

  // Line Number
  int iLine = 1;
  size_t iScalar = 0;

  while(std::getline(fin,line)){
    // Checking if the line is commented
    if(line[0] != '#'){
      if(iLine >= startLine && iLine <= endLine){

        std::stringstream s(line);
        int j = -offset;

        while(std::getline(s, value, ',')){
          if(j>=0 && iScalar<vecOutput.size()){
            vecOutput[iScalar] = stod(value);
            ++iScalar;
          }
          ++j;
        }
      }
      ++iLine;
    }
  }
  fin.close();

  // Verifying we read correct amount of data
  if(iScalar != vecOutput.size()){
    std::cout << "File Data Size: " << iScalar << std::endl;
    std::cout << "Output Vector Size: " << vecOutput.size() << std::endl;
    std::string errMsg = fileName+ " do not have correct data";
    throw std::runtime_error(errMsg);
  }
}

// Writes a vector<double> to a file
void writeVector(const std::vector<double> x, const std::string fileName, std::ios_base::openmode writingMode){
  // Logging the cost function
  std::ofstream logFile;
  logFile << std::fixed << std::setprecision(10) << std::endl;

  logFile.open(fileName, writingMode);
  for(auto i : x){
      logFile << i << ",";
  }
  logFile << "\n";
  logFile.close();
};
