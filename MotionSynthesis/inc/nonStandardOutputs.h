#ifndef OPENSIM_NONSTANDARD_OUTPUTS_H_
#define OPENSIM_NONSTANDARD_OUTPUTS_H_

#include "osimPluginDLL.h"

#include "Component.h"
#include "OpenSim/Simulation/Model/Actuator.h"
#include "OpenSim/Simulation/Model/Model.h"
namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a cylinder for muscle wrapping.
 *
 * @author Peter Loan  
 * updated for OpenSim 4.0 by Benjamin Michaud, 2019.
 */
class OSIMPLUGIN_API NonStandardOutputs : public Component{
OpenSim_DECLARE_CONCRETE_OBJECT(NonStandardOutputs, Component);

OpenSim_DECLARE_OUTPUT(lumbarActuatorActivation, double, getLumbarActuatorActivation,
            SimTK::Stage::Velocity);

public:

//=============================================================================
// METHODS
//=============================================================================
public:
    NonStandardOutputs(){};
    NonStandardOutputs(OpenSim::Model *osimModel);
    virtual ~NonStandardOutputs();
    double getLumbarActuatorActivation(const SimTK::State& s) const;
private:
    const OpenSim::Model *osimModel;
};
} // end of namespace OpenSim

#endif  