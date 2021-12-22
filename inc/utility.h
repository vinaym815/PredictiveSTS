#pragma once

#include <sstream>
#include "Eigen/Dense"
#include "cmaes.h"
#include "universalConsts.h"

using namespace libcmaes;

// reads fileName(delimiter=",") from startLine to endLine into vecOutput
void readVector(std::vector<double> &vecOutput, std::string fileName,
                const int startLine=1, const int endLine=1000, const int offset=0);

void writeVector(const std::vector<double> x, const std::string fileName,
                std::ios_base::openmode writingMode= std::ios::app);

// Creates a timestamp based unique folder in the specified directory
std::string createUniqueFolder(const std::string path);

// Logs the optimization progress
void progressFunc(const CMAParameters<GenoPhenoType> &cmaparams, const CMASolutions &cmasols,
                const std::string logFolder);

CMASolutions resumeDistribution(const std::string dirName, CMAParameters<GenoPhenoType> &cmaparams);