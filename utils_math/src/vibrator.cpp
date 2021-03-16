#include "vibrator.h"

template<typename T>
vibrator<T> *vibrator<T>::me = NULL;

template<typename T>
vibrator<T>::vibrator(T* input, T* output,T magnitude, T decayRate, T frequency, T filterGain) 
: _vibInput(input), _vibOutput(output){
    _vibMagnitude = magnitude; 
    _vibDecayRate = decayRate;
    _vibFrequency = frequency; 
     me = this;
     _vibFilter = new LP_Filterd(filterGain);
     _myStatus = FINISHED;
     _flagTrigger=false;
     _flagReset=false;
     *_vibOutput=0.0;
     *_vibInput=0.0;
     _vibInputInit = *_vibInput;
     _myDuration = ros::Duration(0.0);
}

template<typename T>
vibrator<T>::vibrator(T* input, T* output, T magnitude) : vibrator<T>::vibrator(input, output,magnitude, defaultVibDecayRate, defaultVibFrequency, 0.5) 
{

}

template<typename T>
vibrator<T>::~vibrator()
{
  delete(_vibFilter);
  delete(_vibInput);
  delete(_vibOutput);
}

template<typename T> 
void vibrator<T>::update(ros::Time myCurrentTime)
{
    switch (_myStatus)
        {
            case VIBRATING:
                {
                    _myDuration = (myCurrentTime - _myStartTime);
                    //cout<<_myDuration<<endl;
                    T vibration = 1.0f * _vibMagnitude * exp(-_vibDecayRate * (_myDuration.toSec())) *
                                            sin(2 * M_PI * _vibFrequency * _myDuration.toSec());

                    *_vibOutput = _vibFilter->update(vibration);
                    
                    if ( abs(*_vibOutput) < 0.001f)
                    {
                        _myStatus = FINISHED;
                        _flagTrigger=false;
                    }
                    
                    break;
                }
            case STANDBY:
                {
                    _myStartTime = myCurrentTime;
                    _vibInputInit = *_vibInput;
                    if (_flagTrigger)
                    {
                        _myStatus=VIBRATING;
                    }
                    break;
                }
            case FINISHED:    
                {
                    *_vibOutput = 0.0;
                    _vibInputInit = 0.0;
                    break;
                }
        }  
}

template<typename T>
bool vibrator<T>::run(ros::Time myCurrentTime)
{
    update(myCurrentTime);
    return _flagTrigger;
}

template<typename T>
void vibrator<T>::start()
{
    if(!_flagTrigger)
    {
        _flagTrigger = true;
        _flagReset=false;
    }
}

template<typename T>
void vibrator<T>::reset(){

    if(!_flagReset)
    {
       _myStatus=STANDBY;
       _flagTrigger=false;
       _flagReset=true;
    }
}

template<typename T>
bool vibrator<T>::finished()
{
    return _myStatus==FINISHED;
}


template<typename T>
void vibrator<T>::changeParams(T magnitude, T decayRate, T frequency)
{
    _vibMagnitude = magnitude;
    _vibDecayRate = decayRate;
    _vibFrequency = frequency;
}

template class vibrator<float>;
template class vibrator<double>;