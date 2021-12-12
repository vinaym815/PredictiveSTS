#include "parameterization.h"

// Piecewise Linear Function with fixed dt
/*
Can not parameterize from 0.0s as it enables model to achieve non compressive seat force without
triggering seat release
*/
PWLinearFixedDt::PWLinearFixedDt(SimTK::Vector params){

    int numPoints = params.size();
    const double dT = simulationDuration/(numPoints);
    SimTK::Vector aTimes(numPoints+1);
    SimTK::Vector aValues(numPoints+1);

    for(int i=0; i<numPoints+1; ++i){
        aTimes[i] = i*dT;
        if (i==0){
            aValues[i] = defaultExcitationController;
        }
        aValues[i] = params[i-1];
    }
    pieceWiseLinearFunc.reset(new OpenSim::PiecewiseLinearFunction(numPoints+1, aTimes.getContiguousScalarData(), aValues.getContiguousScalarData(), ""));
}

double PWLinearFixedDt::getValue(const double t) const {
    SimTK::Vector time(1);
    time[0] = t;
    return pieceWiseLinearFunc->calcValue(time);
}