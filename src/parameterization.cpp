#include "parameterization.h"

// Piecewise Linear Function with fixed dt
PWLinearFixedDt::PWLinearFixedDt(SimTK::Vector params){

    int numPoints = params.size();
    const double dT = T_MAX / (numPoints-1);
    SimTK::Vector aTimes(numPoints);
    SimTK::Vector aValues(numPoints);

    for(int i=0; i<numPoints; ++i){
        aTimes[i] = i * dT;
        aValues[i] = params[i];
    }
    pieceWiseLinearFunc.reset(new OpenSim::PiecewiseLinearFunction(numPoints, aTimes.getContiguousScalarData(), 
                            aValues.getContiguousScalarData(), ""));
}

double PWLinearFixedDt::getValue(const double t) const {
    SimTK::Vector time(1);
    time[0] = t;
    return pieceWiseLinearFunc->calcValue(time);
}