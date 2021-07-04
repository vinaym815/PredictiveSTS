#include "parameterization.h"

// Piecewise Linear Function with variable dt
PWLinearVariableDt::PWLinearVariableDt(const SimTK::Vector params){

    // Checking the size of input variable
    if(!(params.size()%2 == 0) ){
        std::cout << "Please check the size of PieceWise Linear parameters vector." << std::endl;
        exit(1);
    }

    int numPoints = params.size()/2; 

    SimTK::Vector aTimes(numPoints);
    SimTK::Vector aValues(numPoints);
    double time = 0;
    for(int i=0; i<numPoints; ++i){
        aTimes[i] = time;
        aValues[i] = params[numPoints+i];
        time += params[i];
    }

    pieceWiseLinearFunc.reset(new OpenSim::PiecewiseLinearFunction(numPoints, aTimes.getContiguousScalarData(), aValues.getContiguousScalarData(), ""));
}

double PWLinearVariableDt::getValue(const double t) const {
    SimTK::Vector time(1);
    time[0] = t;
    return pieceWiseLinearFunc->calcValue(time);
}

// Piecewise Linear Function with fixed dt
PWLinearFixedDt::PWLinearFixedDt(SimTK::Vector params){

    int numPoints = params.size();
    const double dT = simulationDuration/(numPoints-1);
    SimTK::Vector aTimes(numPoints);
    SimTK::Vector aValues(numPoints);

    for(int i=0; i<numPoints; ++i){
        aTimes[i] = i*dT;
        aValues[i] = params[i];
    }

    pieceWiseLinearFunc.reset(new OpenSim::PiecewiseLinearFunction(numPoints, aTimes.getContiguousScalarData(), aValues.getContiguousScalarData(), ""));
}

double PWLinearFixedDt::getValue(const double t) const {
    SimTK::Vector time(1);
    time[0] = t;
    return pieceWiseLinearFunc->calcValue(time);
}