/* -------------------------------------------------------------------------- *
 *  Used to synthesize assisted/Unassisted Sit to Stand and Sitting motions
 *  Look hyperparams____.txt and universalConsts.txt for parameters
  */

/* -------------------------------------------------------------------------- *
* To Do : 1) Auto seat constraint index
*/
//==============================================================================

#include "OpenSim/OpenSim.h"
#include "modelFuns.h"
#include "cmaes.h"
#include "customStrategy.h"
#include "universalConsts.h"

#include <iostream>
#include <mutex>
#include <math.h>
#include <vector>
#include <algorithm>

using namespace libcmaes;

//// Parameterization used for the muscle excitation
const ParameterizationType parameterization = ParameterizationType::PWLinearFixedDt;

// Used to load the human model 
std::mutex threadLock;
const int numThreads = std::thread::hardware_concurrency()-1;
//const int numThreads = 1;

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

        // Resume file Name
        bool resumeFlag = false;
        std::string resumeDir = argv[3];
        if(resumeDir != "null"){
            resumeDir = argv[3];
            resumeFlag = true;
        }

        // Vector containing the hyper parameters
        std::vector<double> hyperParamsVec(numHyperParams);
        readVector(hyperParamsVec, hyperParamFileName);

        //// Population Size
        const int popSize = hyperParamsVec[0]*numThreads;

        //// Maximum number of generations
        const int maxGen = hyperParamsVec[1];

        // Number of components per muscle excitation parameterization
        const int numComps = int(hyperParamsVec[2]);

        // Generation after which relative weights of the cost terms are changed 
        const int costSchedulingGen = int(hyperParamsVec[6]);

        // Number of paramenets for each component of muscle excitation signal parameterization 
        const int numVarsPerComp = mapNumVarsPerComp[parameterization];

        // Number of optimization variables
        const int numDecisionVars = numVarsPerComp*numComps*numExtFuncs;

        // Vector containing the initial guesses. It is overwritten if an resume solution is provided
        std::vector<double> initGuess(numDecisionVars);
        #ifdef Standing
            readVector(initGuess, initialGuessFileName, 1, 1, 2);
        #else
            readVector(initGuess, initialGuessFileName);
        #endif

        //// Standard deviations and bounds for the CMAES algorithm
        std::vector<double> initStepSize(numDecisionVars), upperBound(numDecisionVars), lowerBound(numDecisionVars);
        for(int i=0; i<numExtFuncs; ++i){
            for(int j=0; j<numComps; ++j){
                for(int k=0; k<numVarsPerComp; ++k){
                    initStepSize[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[3+k];
                    lowerBound[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[3+numVarsPerComp+k];
                    upperBound[(numVarsPerComp*i+k)*numComps+j] = hyperParamsVec[3+2*numVarsPerComp+k];
                }
            }
        }

        // Weights used by the cost function
        int offset = numHyperParams-2*numWeights;
        std::vector<double> costWeights(hyperParamsVec.begin()+offset, hyperParamsVec.begin()+offset+numWeights);

        // Setting up the log files and folder
        const std::string logFolder = createUniqueFolder("results/");
        const std::string plotFile = logFolder + "plotting.dat";

        // Adding the geometry directory to the search path
        OpenSim::ModelVisualizer::addDirToGeometrySearchPaths("../../Geometry");
        addComponentsToModel(modelName, newModelName, simulationDuration);

        // The objection function
        FitFunc objectiveFunc = [&numComps, &costWeights](const double* newControls, const int numDecisionVars){
            
            // Reading the human model 
            std::unique_lock<std::mutex> locker(threadLock);
            OpenSim::Model osimModel(newModelName);
            locker.unlock();

            // Running the simulation
            const std::vector<double> costs = runSimulation(osimModel, parameterization, newControls, numComps);

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

        // Set algo do not have any effect because of the custom CMA strategy being used
        CMAParameters<GenoPhenoType> cmaparams(initGuess, initStepSize, popSize, lowerBound, upperBound);
        cmaparams.set_fplot(plotFile);
        cmaparams.set_mt_feval(true);
        cmaparams.set_max_hist(500);
        cmaparams.set_max_iter(maxGen);

        // Initialization is needed before a solution can be created from it
        cmaparams.initialize_parameters();

        ESOptimizer<customStrategy<ACovarianceUpdate, GenoPhenoType>,CMAParameters<GenoPhenoType>> *optimizer;

        // Reloading gaussian distribution in case the optimization is resumed 
        if(resumeFlag){
            CMASolutions resumeSolution = resumeDistribution(resumeDir, cmaparams);
            optimizer = new ESOptimizer<customStrategy<ACovarianceUpdate, GenoPhenoType>,CMAParameters<GenoPhenoType>> (objectiveFunc, cmaparams, resumeSolution);
        }
        else{
            optimizer = new ESOptimizer<customStrategy<ACovarianceUpdate, GenoPhenoType>,CMAParameters<GenoPhenoType>> (objectiveFunc, cmaparams);
        }
        optimizer->set_progress_func(writeLogs);

        int iter = 1;
        while((!optimizer->stop() || iter<maxGen)){
            dMat candidates = optimizer->ask();
            optimizer->eval(candidates,optimizer->get_parameters().get_gp().pheno(candidates));
            optimizer->tell();
            optimizer->inc_iter();

            // Updating the relative weights of the cost function
            if(iter == costSchedulingGen){
                offset = numHyperParams-numWeights;
                costWeights.assign(hyperParamsVec.begin()+offset, hyperParamsVec.begin()+offset+numWeights);
                CMASolutions &cmasols = optimizer->get_solutions();
                cmasols.get_best_seen_candidate().set_fvalue(1e10);
                cmasols.set_sigma(0.1);
            }

            ++iter;
        }

        delete optimizer;

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