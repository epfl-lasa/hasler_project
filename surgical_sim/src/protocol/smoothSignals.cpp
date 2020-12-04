#include "smoothSignals.h"


smoothSignals *smoothSignals::me = NULL;

smoothSignals::smoothSignals(smoothSignals_Type type, double* output, double duration) 
: _myType(type),_signalOutput(output), _myDuration(duration){

     me = this;
     _myStatus = STANDBY;
     _flagTrigger=false;
     _flagReset=false;
     _durationBias=0.0;
     _myElapsedTime = ros::Duration(0.0);
}

smoothSignals::~smoothSignals()
{
  delete(_signalOutput);
}
 
void smoothSignals::update(ros::Time myCurrentTime)
{
    
    switch (_myStatus)
        {
            case CHANGING:
                {
                    _myElapsedTime = (myCurrentTime - _myStartTime);

                    switch (_myType)
                    {
                    case SMOOTH_FALL:
                    {
                        *_signalOutput = Utils_math<double>::smoothFall(_myElapsedTime.toSec(),0,_myDuration);
                        if (_myElapsedTime.toSec()>=_myDuration)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                    }
                    
                    case SMOOTH_RISE:
                    {
                        *_signalOutput = Utils_math<double>::smoothRise(_myElapsedTime.toSec(),0,_myDuration);
                        if (_myElapsedTime.toSec()>=_myDuration)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                    }

                      case SMOOTH_RISE_FALL:
                    {
                        *_signalOutput = Utils_math<double>::smoothRiseFall(_myElapsedTime.toSec(),0,_myDuration/2.0, _myDuration/2.0,_myDuration);
                        if (_myElapsedTime.toSec()>=_myDuration)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                    }

                    case SMOOTH_RISE_CUT:
                    {
                        if (*_signalOutput<=0.0001)
                        {
                            *_signalOutput = Utils_math<double>::smoothRise(_myElapsedTime.toSec(),0,500.0*_myDuration);
                            _durationBias = _myElapsedTime.toSec();
                        }
                        else
                        {
                            *_signalOutput = 0.001 + Utils_math<double>::smoothRise(_myElapsedTime.toSec(),_durationBias,_durationBias+_myDuration);
                            if (_myElapsedTime.toSec()>=_myDuration + _durationBias)
                            {
                              _myStatus = FINISHED;
                              _flagTrigger=false;
                            }
                        }
                        
                        break;
                    }

                    default:
                        break;
                    }
       
                
                    *_signalOutput = Utils_math<double>::bound(*_signalOutput,0.0,1.0);

                    break;
                }
            case STANDBY:
                {
                    _myStartTime = myCurrentTime;
                    if (_flagTrigger)
                    {
                        _myStatus=CHANGING;
                    }
                    break;
                }
            case FINISHED:    
                {
                    break;
                }
        }  
}

bool smoothSignals::run(ros::Time myCurrentTime)
{
    update(myCurrentTime);
    return _flagTrigger;
}

void smoothSignals::start()
{
    if(!_flagTrigger)
    {
        _flagTrigger = true;
        _flagReset = false;
    }
}

void smoothSignals::reset(){

    if(!_flagReset)
    {
       _myStatus=STANDBY;
       _flagTrigger=false;
       _flagReset=true;
    }
}

bool smoothSignals::finished()
{
    return _myStatus==FINISHED;
}


void smoothSignals::changeParams(smoothSignals_Type type, double duration)
{
    _myDuration=duration;
    _myType=type;
    switch (_myType)
    {
    case SMOOTH_FALL:
        if (fabs(*_signalOutput-0.0)<=FLT_EPSILON)
        {   
            *_signalOutput=0.0;
            _myStatus=FINISHED;
        }
        break;
    
     case SMOOTH_RISE:
        if (fabs(*_signalOutput-1.0)<=FLT_EPSILON)
        {
            *_signalOutput=1.0;
            _myStatus=FINISHED;
        }
        break;
    }

    
}