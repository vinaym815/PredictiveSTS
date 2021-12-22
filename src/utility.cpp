#include "utility.h"

// Saves the log and resume files
void progressFunc(const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols, const std::string logFolder){
    using namespace libcmaes;
    Eigen::VectorXd xMean = cmasols.xmean();
    std::vector<double> xMeanVector(xMean.data(), xMean.data() + xMean.rows() * xMean.cols());
    writeVector(xMeanVector, logFolder+RESUME_MEAN_FILE , std::ios::out);

    Eigen::MatrixXd covMat = cmasols.cov();
    Eigen::Map<Eigen::VectorXd> covMatFlat(covMat.data(), covMat.size());
    std::vector<double> covVec(covMatFlat.data(), covMatFlat.data() + covMatFlat.rows() * covMatFlat.cols());
    writeVector(covVec, logFolder+RESUME_COV_FILE, std::ios::out);

    std::vector<double> sigmaVec(1);
    sigmaVec[0] = cmasols.sigma();
    writeVector(sigmaVec, logFolder+RESUME_SIGMA_FILE, std::ios::out);

    std::vector<double> vecOutBest;
    vecOutBest.push_back(cmasols.niter());
    vecOutBest.push_back(cmasols.best_candidate().get_fvalue());
    Eigen::VectorXd bestparameters = cmaparams.get_gp().pheno(cmasols.best_candidate().get_x_dvec());
    vecOutBest.insert(vecOutBest.end(), bestparameters.data(),
                     bestparameters.data() + bestparameters.rows() * bestparameters.cols());

    writeVector(vecOutBest, logFolder + LOG_BEST_FILE);
};

// Loads the mean, stdDev and sigma from files
CMASolutions resumeDistribution(const std::string dirName, CMAParameters<GenoPhenoType> &cmaparams){

    std::cout << "Resuming Distribution" << std::endl;
    std::vector<double> resumeMeanVec(cmaparams.dim());
    readVector(resumeMeanVec, dirName + RESUME_MEAN_FILE);
    Eigen::Map<Eigen::VectorXd> resumeXMean(resumeMeanVec.data(), resumeMeanVec.size());

    std::vector<double> resumeCovVec(cmaparams.dim()*cmaparams.dim());
    readVector(resumeCovVec, dirName+RESUME_COV_FILE);
    Eigen::MatrixXd resumeCovMat = Eigen::Map<Eigen::MatrixXd>(resumeCovVec.data(), 
                                                cmaparams.dim(), cmaparams.dim());

    std::vector<double> resumeSigmaVec(1);
    readVector(resumeSigmaVec, dirName + RESUME_SIGMA_FILE);
    double sigma = resumeSigmaVec[0];

    CMASolutions resumeSolution(cmaparams);
    resumeSolution.set_xmean(resumeXMean);
    resumeSolution.set_sigma(sigma);
    resumeSolution.set_cov(resumeCovMat);

    return resumeSolution;
}

std::string createUniqueFolder(const std::string path){
    const std::chrono::seconds timeStamp = std::chrono::duration_cast<std::chrono::seconds>
                                            (std::chrono::system_clock::now().time_since_epoch());

    const std::string logFolder = path + std::to_string(timeStamp.count())+"/";
    mkdir(path.c_str(), 0777);
    mkdir(logFolder.c_str(), 0777);
    return logFolder;
};

// Used to read a set of lines from a file
void readVector(std::vector<double> &vecOutput, std::string fileName, const int startLine, 
                const int endLine, const int offset){

    std::fstream fin;
    fin.open(fileName, std::ios::in);
    std::string line, value;
    int iLine = 1;
    size_t iScalar = 0;
    while(std::getline(fin,line)){
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

    if(iScalar != vecOutput.size()){
        std::cout << "File Data Size: " << iScalar << std::endl;
        std::cout << "Output Vector Size: " << vecOutput.size() << std::endl;
        std::string errMsg = fileName+ " do not have correct data";
        throw std::runtime_error(errMsg);
    }
}

// Writes a vector<double> to a file
void writeVector(const std::vector<double> x, const std::string fileName, std::ios_base::openmode writingMode){
    std::ofstream logFile;
    logFile.open(fileName, writingMode);
    for(auto i : x){
        logFile << i << ",";
    }
    logFile << "\n";
    logFile.close();
};