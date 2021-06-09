#include "footVarSynchronizer.h"

footVarSynchronizer *footVarSynchronizer::me = NULL;

footVarSynchronizer::footVarSynchronizer(ros::NodeHandle &n_1, float frequency,footVarSynchronizer::Platform_Name platform_id): 
_n(n_1),
_platform_name(platform_id),
_loopRate(frequency),
_dt(1.0f/frequency)

{
	_flagPIDGainsByInput=false;
	me=this;
	_stop = false;
	

}
footVarSynchronizer::~footVarSynchronizer()
{
	me->_n.shutdown();
}


