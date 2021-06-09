#include "footVarSynchronizer.h"

void footVarSynchronizer::correctForceForLegCompensation() {
	 
	 if (_flagHumanOnPlatform && _flagCompensateLeg && _flagLegCompWrenchRead && _subLegGravCompWrench.getNumPublishers()!=0)
	 {
		_ros_forceSensor_controlled = _ros_forceModified;// + _legWrenchGravityComp;                               
	 }

	 else
	 {
		_ros_forceSensor_controlled = _ros_forceModified;
	 }

}

