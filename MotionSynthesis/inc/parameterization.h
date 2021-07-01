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
Sum of Gaussians Parameterization
Format: A1, A2, A3, ... An, B1, B2, B3, ..., Bn, C1, C2, C3, ..., Cn
*/
class GMM : public CustomFunction{
    public:
        constexpr static int nVarsPerComp = 3;
        GMM(const SimTK::Vector params);
        ~GMM(){};
        double getValue(const double t) const override;
    private:
        const SimTK::Vector params;
};

/*
Piecewise Linear Parameterization with variable Dt
Format: dt1, dt2,...dtn, P1, P2, P3...,Pn
Bug: Last dt i.e dtn is not being used
   : Not modified as it would indexing modification of initStepSize, upperBound and Lower Bound
*/
class PWLinearVariableDt : public CustomFunction{
    public:
        constexpr static int nVarsPerComp = 2;
        PWLinearVariableDt(const SimTK::Vector params);
        ~PWLinearVariableDt(){};
        double getValue(const double t) const override;
    private:
        std::unique_ptr<OpenSim::PiecewiseLinearFunction> pieceWiseLinearFunc;
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
    GMM,
    PWLinearVariableDt,
    PWLinearFixedDt
};

/*
Number of paramerts required by each component of a parameterization
*/
static std::map<ParameterizationType, int> mapNumVarsPerComp{
    {ParameterizationType::GMM, GMM::nVarsPerComp},
    {ParameterizationType::PWLinearVariableDt, PWLinearVariableDt::nVarsPerComp},
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
    {ParameterizationType::GMM, createExtFuncPtr<GMM>},
    {ParameterizationType::PWLinearVariableDt, createExtFuncPtr<PWLinearVariableDt>},
    {ParameterizationType::PWLinearFixedDt, createExtFuncPtr<PWLinearFixedDt>}
};