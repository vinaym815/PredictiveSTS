#include "nonStandardOutputs.h"

using namespace std;
using namespace OpenSim;

NonStandardOutputs::NonStandardOutputs(OpenSim::Model *osimModel) : Component(), osimModel(osimModel)
{}

//_____________________________________________________________________________
/*
* Destructor.
*/
NonStandardOutputs::~NonStandardOutputs()
{}

double NonStandardOutputs::getLumbarActuatorActivation(const SimTK::State &s) const {
    const OpenSim::ScalarActuator* scalarActuator=  dynamic_cast<const OpenSim::ScalarActuator*>
                                                               (&osimModel->getComponent("/forceset/lumbarCoordinateActuator"));
    if(scalarActuator == NULL){
        std::string errMsg =  "could not get a handle on scalar actuator";
        std::__throw_runtime_error(errMsg.c_str());
    }
    return scalarActuator->getActuation(s)/scalarActuator->getOptimalForce();
};