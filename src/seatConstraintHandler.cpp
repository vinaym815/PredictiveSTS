#include "seatConstraintHandler.h"

ReleaseSeatConstraint::ReleaseSeatConstraint(OpenSim::Model& m, const SimTK::ConstraintIndex index, const double threshold) : TriggeredEventHandler(SimTK::Stage::Acceleration),
		_model(m), _forceThreshold(threshold),  _index(index) 
	{
		getTriggerInfo().setTriggerOnRisingSignTransition(false);
		getTriggerInfo().setTriggerOnFallingSignTransition(true);
	}

	// WITNESS FUNCTION
SimTK::Real ReleaseSeatConstraint::getValue(const SimTK::State& s) const{

	SimTK::Constraint &seatConstraint = _model.updMatterSubsystem().updConstraint(_index);
	if(seatConstraint.isDisabled(s)){
		return -1;
	}

	const int ncb = seatConstraint.getNumConstrainedBodies();
	const int ncm = seatConstraint.getNumConstrainedU(s);

	SimTK::Vector_<SimTK::SpatialVec> bodyForcesInAncestor(ncb);
    bodyForcesInAncestor.setToZero();
    SimTK::Vector mobilityForces(ncm, 0.0);

	SimTK::Vector multipliers = seatConstraint.getMultipliersAsVector(s);
	seatConstraint.calcConstraintForcesFromMultipliers(s, multipliers, bodyForcesInAncestor, mobilityForces);

    OpenSim::Array<double> values(0.0,6*ncb+ncm);
	for(int i=0; i<ncb; ++i){
        for(int j=0; j<3; ++j){
            // Simbody constraints have reaction moments first and OpenSim reports forces first
            // so swap them here
            values[i*6+j] = (bodyForcesInAncestor(i)[1])[j]; // moments on constrained body i
            values[i*6+3+j] = (bodyForcesInAncestor(i)[0])[j]; // forces on constrained body i
        }
    }
	for(int i=0; i<ncm; ++i){
        values[6*ncb+i] = mobilityForces[i];
    }
	const double forceOnSeatFromGroundX = values[0];
	const double forceOnSeatFromGroundY = values[1];
	const double signal = std::min(forceOnSeatFromGroundY, MU_STATIC * forceOnSeatFromGroundY) - fabs(forceOnSeatFromGroundX);

	//std::cout << s.getTime() << " : " << forceOnSeatFromGroundY - _forceThreshold <<  
	//							", "<< MU_STATIC*forceOnSeatFromGroundY-fabs(forceOnSeatFromGroundX) 
	//								<< std::endl;
	return signal - _forceThreshold;
}

	// EVENT HANDLER FUNCTION
void ReleaseSeatConstraint::handleEvent(SimTK::State& s, SimTK::Real accuracy, bool& terminate) const
{
	_model.updMatterSubsystem().updConstraint(_index).disable(s);
	terminate = false;
	seatReleaseTime = s.getTime(); 
	std::cout << "Releasing Seat Constraint : " << seatReleaseTime << std::endl;
}

double ReleaseSeatConstraint::getSeatRleaseTime() const{
	return seatReleaseTime;
}