/* -------------------------------------------------------------------------- *
 *  Used to synthesize assisted/Unassisted Sit to Stand and Sitting motions
 *  Look hyperparams____.txt and universalConsts.txt files for parameters
  */

/* -------------------------------------------------------------------------- *
* To Do : 1) Automate seat constraint index
*/
//==============================================================================

#include "OpenSim/OpenSim.h"
#include "modelFuns.h"
#include "cmaes.h"
#include "universalConsts.h"

#include <iostream>
#include <mutex>
#include <math.h>
#include <vector>
#include <algorithm>

using namespace libcmaes;

//// Parameterization used for the muscle excitation trajectories
const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;

// Used to load the human model 
std::mutex threadLock;

// Main Function
int main(int argc, const char *argv[])
{
    try {
        if (argc != 4){
            std::cout << "Inappropriate number of function arguments" << std::endl;
            std::cout << "Correct Format: funcName.exe initialGuessFile.txt hyperParametersFile.txt resumeDirectory/null)" 
                      << std::endl;
            exit(1);
        }

        const std::string initialGuessFileName = argv[1]; 
        const std::string hyperParamFileName = argv[2];

        bool resumeFlag = false;
        std::string resumeDir = argv[3];
        if(resumeDir != "null"){
            resumeDir = argv[3];
            resumeFlag = true;
        }

        // Vector containing the hyper parameters
        std::vector<double> hyperParamsVec(numHyperParams);
        readVector(hyperParamsVec, hyperParamFileName);

        // Number of variables 
        const int numComps = int(hyperParamsVec[0]);
        const int numVarsPerComp = mapNumVarsPerComp[parameterization];
        const int numDecisionVars = numVarsPerComp*numComps*numExtFuncs+1;

        // The initial guess (mean) is overwritten if an resume solution is provided
        std::vector<double> initGuess(numDecisionVars);
        #ifdef Standing
            readVector(initGuess, initialGuessFileName, 1, 1, 2);
        #else
            readVector(initGuess, initialGuessFileName);
        #endif

        //// Standard deviations and bounds for the CMAES algorithm
        std::vector<double> initStepSize(numDecisionVars), upperBound(numDecisionVars), lowerBound(numDecisionVars);
        readVector(initStepSize,"stdDev.txt");
        readVector(lowerBound,"lowerBound.txt");
        readVector(upperBound,"upperBound.txt");
        //for(int i=0; i<numExtFuncs; ++i){
        //    for(int j=0; j<numComps; ++j){
        //        for(int k=0; k<numVarsPerComp; ++k){
        //            initStepSize[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[1+k];
        //            lowerBound[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[1+numVarsPerComp+k];
        //            upperBound[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[1+2*numVarsPerComp+k];
        //        }
        //    }
        //}

        // Weights used by the cost function
        int offset = numHyperParams-numWeights;
        std::vector<double> costWeights(hyperParamsVec.begin()+offset, hyperParamsVec.begin()+offset+numWeights);

        // Setting up the log files and folder
        const std::string logFolder = createUniqueFolder("results/");
        const std::string plotFile = logFolder + "plotting.dat";

        // Adding the geometry directory to the search path
        OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../geometry");

        // Adds a prescribed controller with piecewise linear functions to the model and saves a copy of it 
        addComponentsToModel(modelName, newModelName, simulationDuration);

        // The objection function
        FitFunc objectiveFunc = [&numComps, &costWeights](const double* newControls, const int numDecisionVars){
            
            // Reading the human model 
            std::unique_lock<std::mutex> locker(threadLock);
            OpenSim::Model osimModel(newModelName);
            locker.unlock();

            // Running the forward simulation
            const std::vector<double> costs = runSimulation(osimModel, parameterization, numDecisionVars, newControls, numComps);

            // Computing the weighted cost
            const double value = std::inner_product(costWeights.begin(), costWeights.end(), costs.begin(), 0.0);
            return value;
        };

        // Progress function used to write logs and resumption files
        ProgressFunc<CMAParameters<GenoPhenoType>,CMASolutions> writeLogs = [&logFolder]
                    (const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols){
            progressFunc(cmaparams, cmasols, logFolder);
            return 0;
        };

        // CMA parameters
        CMAParameters<GenoPhenoType> cmaparams(initGuess, initStepSize, -1, lowerBound, upperBound);
        cmaparams.set_algo(aCMAES);
        cmaparams.set_fplot(plotFile);
        cmaparams.set_max_hist(histSize);
        cmaparams.set_ftolerance(fTolerance);
        cmaparams.set_max_iter(maxIter);
        cmaparams.set_noisy();
        cmaparams.set_mt_feval(true);

        // Running the optimization nRestart times
        CMASolutions cmasols;
        for(int i=0; i<nRestarts; ++i){
            CMASolutions cmasols_temp;
            auto seed = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>
                                                (std::chrono::high_resolution_clock::now().time_since_epoch()).count());
            cmaparams.set_seed(seed);
            cmaparams.initialize_parameters();

            if(i==0 && resumeFlag){
                    CMASolutions resumeSolution = resumeDistribution(resumeDir, cmaparams);
                    cmasols_temp = cmaes<GenoPhenoType>(objectiveFunc, cmaparams, writeLogs, nullptr, resumeSolution);
            }
            else{
                    cmasols_temp = cmaes<GenoPhenoType>(objectiveFunc, cmaparams, writeLogs);
            }

            if(cmasols.candidates().empty() || cmasols_temp.get_best_seen_candidate().get_fvalue() < cmasols.get_best_seen_candidate().get_fvalue())
                cmasols = cmasols_temp;
        }

        // Saving the results from best simulation
        Eigen::VectorXd optimParams = cmaparams.get_gp().pheno(cmasols.get_best_seen_candidate().get_x_dvec());
        OpenSim::Model osimModel(newModelName);
        std::vector<double> finalCosts = runSimulation(osimModel, parameterization, numDecisionVars, optimParams.data(), numComps, true, true, "optimResults");
        const double totalCost = std::inner_product(finalCosts.begin(), finalCosts.end(), costWeights.begin(), 0.0); 
        std::cout << "Optimized Cost : " << totalCost << std::endl;

        // Exit message
        std::cout << "Finished Optimization" << std::endl;

    }
    catch(const std::string& ex){
        std::cout << ex << std::endl;
        return 1;
    }
    catch (const std::exception& ex){
        std::cout << ex.what() << std::endl;
        return 1;
    }

    return 0;
}