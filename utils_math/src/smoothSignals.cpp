#include "smoothSignals.h"



template<typename T>
smoothSignals<T> *smoothSignals<T>::me = NULL;

template<typename T>
smoothSignals<T>::smoothSignals(smoothSignals_Type type, T* output, T timeFreq) 
: _myType(type),_signalOutput(output), _myTimeFreq(timeFreq){

     me = this;
     _myStatus = STANDBY;
     _flagTrigger=false;
     _flagReset=false;
     _durationBias=0.0;
     _myElapsedTime = ros::Duration(0.0);
}

template<typename T>
smoothSignals<T>::~smoothSignals()
{
  delete(_signalOutput);
}
template<typename T> 
void smoothSignals<T>::update(ros::Time myCurrentTime)
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
                      
                        *_signalOutput = Utils_math<T>::smoothFall(_myElapsedTime.toSec(),0,_myTimeFreq);
                        if (_myElapsedTime.toSec()>=_myTimeFreq)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                    }
                    
                    case SMOOTH_RISE:
                    {
                      
                        *_signalOutput = Utils_math<T>::smoothRise(_myElapsedTime.toSec(),0,_myTimeFreq);
                        if (_myElapsedTime.toSec()>=_myTimeFreq)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                    }

                      case SMOOTH_RISE_FALL:
                    {
                      
                        *_signalOutput = Utils_math<T>::smoothRiseFall(_myElapsedTime.toSec(),0,_myTimeFreq/2.0, _myTimeFreq/2.0,_myTimeFreq);
                        if (_myElapsedTime.toSec()>=_myTimeFreq)
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
                            *_signalOutput = Utils_math<T>::smoothRise(_myElapsedTime.toSec(),0,500.0*_myTimeFreq);
                            _durationBias = _myElapsedTime.toSec();
                        }
                        else
                        {
                            *_signalOutput = 0.001 + Utils_math<T>::smoothRise(_myElapsedTime.toSec(),_durationBias,_durationBias+_myTimeFreq);
                            if (_myElapsedTime.toSec()>=_myTimeFreq + _durationBias)
                            {
                              _myStatus = FINISHED;
                              _flagTrigger=false;
                            }
                        }
                        
                        break;
                    }
                    case SINUSOID:
                    {   
                      
                        *_signalOutput = std::sin(2.0f*M_PI*_myTimeFreq*_myElapsedTime.toSec() + M_PI_2);
                        if (_myTimeFreq<1.0f)
                        {
                            _myStatus = FINISHED;
                            _flagTrigger=false;
                        }
                        break;
                        
                    }

                    default:
                        break;
                    }
       
                    if(_myType==SINUSOID)
                    {
                        *_signalOutput = Utils_math<T>::bound(*_signalOutput,-1.0,1.0);
                    }else
                    {
                        *_signalOutput = Utils_math<T>::bound(*_signalOutput,0.0,1.0);
                    }

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
template<typename T>
bool smoothSignals<T>::run(ros::Time myCurrentTime)
{
    update(myCurrentTime);
    return _flagTrigger;
}
template<typename T>
void smoothSignals<T>::start()
{
    if(!_flagTrigger)
    {
        _flagTrigger = true;
        _flagReset = false;
    }
}
template<typename T>
void smoothSignals<T>::reset(){

    if(!_flagReset)
    {
       _myStatus=STANDBY;
       _flagTrigger=false;
       _flagReset=true;
    }
}
template<typename T>
bool smoothSignals<T>::finished()
{
    return _myStatus==FINISHED;
}

template<typename T>
void smoothSignals<T>::changeParams(smoothSignals_Type type, T timeFreq)
{
    _myTimeFreq=timeFreq;
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

template class smoothSignals<float>;
template class smoothSignals<double>;