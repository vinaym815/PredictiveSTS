#pragma once

#include "OpenSim/Simulation/Model/ModelComponent.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "activationPointActuator.h"
#include "osimPluginDLL.h"

namespace OpenSim {

class Model;

//Assumes force 1 and force 2 are acting at same point

class OSIMPLUGIN_API AssistanceForceVisualization : public ModelComponent {
OpenSim_DECLARE_CONCRETE_OBJECT(AssistanceForceVisualization, ModelComponent);

public :
    AssistanceForceVisualization(){};

    ~AssistanceForceVisualization(){};

    void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
                            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override{

        Super::generateDecorations(fixed, hints, state, geometry);
        if (!fixed && (state.getSystemStage() >= SimTK::Stage::Dynamics) && hints.get_show_forces()){
            const Model &model = getModel();
            const OpenSim::ActivationPointActuator *force1 = dynamic_cast<const OpenSim::ActivationPointActuator*>(&model.getActuators().get("assistFx"));
            const OpenSim::ActivationPointActuator *force2 = dynamic_cast<const OpenSim::ActivationPointActuator*>(&model.getActuators().get("assistFy"));

            // Point of force application
            SimTK::Vec3 forcePoint = force1->get_point();
            if (!force1->get_point_is_global()){
                forcePoint = model.getBodySet().get(force1->get_body()).findStationLocationInGround(state, forcePoint);
            }

            SimTK::Vec3 dirCos1 = SimTK::UnitVec3(force1->get_direction());
            if (!force1->get_force_is_global()){
                dirCos1 = model.getBodySet().get(force1->get_body()).expressVectorInGround(state, dirCos1);
            }

            SimTK::Vec3 dirCos2 = SimTK::UnitVec3(force2->get_direction());
            if (!force2->get_force_is_global()){
                dirCos2 = model.getBodySet().get(force2->get_body()).expressVectorInGround(state, dirCos2);
            }

            SimTK::Vec2 maxForce {force1->getOptimalForce(), force2->getOptimalForce()};
            SimTK::Vec3 actuationForce = force1->getActuation(state)*dirCos1 + force2->getActuation(state)*dirCos2;
            actuationForce = actuationForce/maxForce.norm();
            const SimTK::Real length(actuationForce.norm());

            const SimTK::Transform forceVizTransform( SimTK::Rotation(SimTK::UnitVec3(actuationForce), SimTK::YAxis), forcePoint + actuationForce/2 );

            // Construct the force decoration and add it to the list of geometries.
            SimTK::DecorativeCylinder forceViz(0.01, 0.5 * length);
            forceViz.setTransform(forceVizTransform);
            forceViz.setColor(SimTK::Vec3(0.0, 0.6, 0.0));
            geometry.push_back(forceViz);
        }

   }
};
}