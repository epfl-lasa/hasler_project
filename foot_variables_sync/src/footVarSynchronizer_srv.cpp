#include "footVarSynchronizer.h"

void footVarSynchronizer::requestSetState(){
	//_mutex.lock();
	_flagSetStateRequested=true;
	_srvSetState.request.ros_machineState=_ros_newState;
	_srvSetState.request.ros_effortComp.resize(NB_EFFORT_COMPONENTS);
	for (int j=0; j<NB_EFFORT_COMPONENTS; j++)
	{
		_srvSetState.request.ros_effortComp[j]=_ros_effortComp[j];
	}

	_clientSetState.call(_srvSetState);

	_flagResponseSetState = _srvSetState.response.platform_newState;
	//_mutex.unlock();
}

void footVarSynchronizer::requestSetController(){
	_flagSetControllerRequested=true;
	_srvSetController.request.ros_controllerType=_ros_controllerType;
	_srvSetController.request.ros_defaultControl=_ros_defaultControl;
	_srvSetController.request.ros_controlledAxis=_ros_controlledAxis;

	_clientSetController.call(_srvSetController);
	ROS_INFO("[%s footVarSync]: Set Controller Request",Platform_Names[_platform_name]);
	_flagResponseSetController = _srvSetController.response.platform_controlOk;

}