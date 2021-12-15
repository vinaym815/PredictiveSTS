// Used to replay and save the optimization results

#include <iostream>
#include <string>
#include <memory>
#include <future>
#include <mutex>

#include "OpenSim/OpenSim.h"
#include "parameterization.h"
#include "modelFuns.h"
#include "cmaes.h"
#include "universalConsts.h"

std::mutex threadLock;

const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;

int main(int argc, char *argv[]){

  // Checking the number of inputs;
  if (argc!=7){
      std::cout << "Inappropriate Number of Arguments" << std::endl;
      std::cout << "Correct Format:\n funcName #ExcitationSegments fileName startGen endGen visualize(true/false) saveData(true/false)" 
                << std::endl;
      exit(1);
  }

  // Number of variables per component
  int numVarsPerComp = mapNumVarsPerComp[parameterization];

  // Number of segments the excitation profile is made up of 
  int numComps = std::stoi(argv[1]);

  // Name of the file containing excitation profile
  const char *fileName = argv[2];

  // Generation Numbers
  int genStart = std::stoi(argv[3]);
  int genEnd = std::stoi(argv[4]);

  bool visualizeResults= false;
  if ((std::string)argv[5] == "true"){
    visualizeResults = true;
  }

  bool saveResults= false;
  if ((std::string)argv[6] == "true"){
    saveResults = true;
  }

  #ifdef Standing
    const std::vector<double> costWeights{600, 0.2, 0.2, 0.4, 100, 40, 10, 40, 0.0, 0.1, 0.2};
  #else
    const std::vector<double> costWeights{5, 4, 200, 40, 10, 20, 0.0, 1.0, 1.0};
  #endif

  OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry");
  addComponentsToModel(modelName, newModelNameReplay, simulationDuration);
  const int numTotalVars = numVarsPerComp*numComps*numExtFuncs+1;

  std::vector<int> gens(genEnd-genStart+1);
  std::iota(gens.begin(), gens.end(), 0);
  std::vector<std::future<void>> futuresVec;

  std::launch policy = std::launch::async;
  if (saveResults || visualizeResults)
    policy = std::launch::deferred;

  for (int gen : gens){
    futuresVec.push_back(std::async(policy, 
      [&](int i){
        std::vector<double> inVec(numTotalVars+2);
        std::unique_lock<std::mutex> locker(threadLock);
          OpenSim::Model osimModel(newModelNameReplay);
          readVector(inVec, fileName, i+genStart, i+genStart, 0);
        locker.unlock();

        const std::string fileNamePrefix = std::to_string(int(inVec[0]));
        const std::vector<double> costs = runSimulation(osimModel, parameterization, numTotalVars, inVec.data()+2, 
                                                        numComps, visualizeResults, saveResults, fileNamePrefix);

        std::vector<double> relativeCosts(costs.size());
        std::transform(costs.begin(), costs.end(), costWeights.begin(), relativeCosts.begin(),std::multiplies<double>());
        const double totalCost = std::accumulate(relativeCosts.begin(), relativeCosts.end(), 0.0); 

        if(visualizeResults){
          locker.lock();
            std::cout << int(inVec[0]) << " : " << totalCost << std::endl;
            for(int i=0; i<relativeCosts.size(); ++i){
              std::cout << relativeCosts[i] << " : " << costs[i] << std::endl;
            }
            std::cout <<std::endl;
          locker.unlock();
        }
        else{
          std::vector<double> vecOut;
          vecOut.push_back(inVec[0]);
          vecOut.push_back(totalCost);
          //vecOut.insert(vecOut.end(), inVec.begin(), inVec.begin()+2);
          vecOut.insert(vecOut.end(), costs.begin(), costs.end());
          locker.lock();
            writeVector(vecOut, "costsData.csv", std::ios::app);
          locker.unlock();
        }
      }, gen
    )
    );
  }
  for(auto &e : futuresVec){
    e.get();
  }

  // Bye bye
  std::cout << "C++: Done!" << std::endl;

  return 0;
}