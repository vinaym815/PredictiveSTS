#pragma once

#include <memory>
#include <map>
#include <functional>
#include "universalConsts.h"
#include "OpenSim/OpenSim.h"

/*
Base class for all the excitation signal parameterization
*/
class CustomFunction{
    public:
        CustomFunction(){};
        virtual ~CustomFunction(){};
        virtual double getValue(double t) const = 0;
};

/*
Piecewise Linear Parameterization with fixed Dt
Format: P0, P1, P2, P3 ............Pn
*/
class PWLinearFixedDt : public CustomFunction{
    public:
        constexpr static int nVarsPerComp = 1;
        PWLinearFixedDt(const SimTK::Vector params);
        ~PWLinearFixedDt(){};
        double getValue(const double t) const override;

    private:
        std::unique_ptr<OpenSim::PiecewiseLinearFunction> pieceWiseLinearFunc;
};

/*
Types of Parameterization
*/
enum class ParameterizationType {
    PWLinearFixedDt
};

/*
Number of paramerts required by each component of a parameterization
*/
static std::map<ParameterizationType, int> mapNumVarsPerComp{
    {ParameterizationType::PWLinearFixedDt, PWLinearFixedDt::nVarsPerComp}
};

// Needed by the parameterization map 
template <class TParameterization>
std::unique_ptr<CustomFunction> createExtFuncPtr(const SimTK::Vector params){
    return std::make_unique<TParameterization>(params);
};

/*
Map to functions that creates parameterization
*/
static std::map<ParameterizationType, std::function<std::unique_ptr<CustomFunction>(const SimTK::Vector)>> mapExtFuncPtr{
    {ParameterizationType::PWLinearFixedDt, createExtFuncPtr<PWLinearFixedDt>}
};