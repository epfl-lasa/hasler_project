#include "vibrator.h"
const double defaultVibMagnitude = -240.0; //![N / m/s]
const double defaultVibDecayRate = 60; //! [1/s]
const double defaultVibFrequency = 32.6; //! [Hz]


vibrator *vibrator::me = NULL;

vibrator::vibrator(double* input, double* output,double magnitude, double decayRate, double frequency, double filterGain) 
: _vibInput(input), _vibOutput(output){
    _vibMagnitude = magnitude; 
    _vibDecayRate = decayRate;
    _vibFrequency = frequency; 
     me = this;
     _vibFilter = new LP_Filterd(filterGain);
     _myStatus = STANDBY;
     _flagTrigger=false;
     _flagReset=false;
     *_vibOutput=0.0;
     *_vibInput=0.0;
     _vibInputInit = *_vibInput;
     _myDuration = ros::Duration(0.0);
}

vibrator::vibrator(double* input, double* output, double magnitude) : vibrator::vibrator(input, output,magnitude, defaultVibDecayRate, defaultVibFrequency, 0.5) 
{

}

vibrator::~vibrator()
{
  delete(_vibFilter);
  delete(_vibInput);
  delete(_vibOutput);
}
 
void vibrator::update(ros::Time myCurrentTime)
{
    switch (_myStatus)
        {
            case VIBRATING:
                {
                    _myDuration = (myCurrentTime - _myStartTime);
                    //cout<<_myDuration<<endl;
                    double vibration = _vibInputInit * _vibMagnitude * exp(-_vibDecayRate * (_myDuration.toSec())) *
                                            sin(2 * M_PI * _vibFrequency * _myDuration.toSec());

                    *_vibOutput = _vibFilter->update(vibration);
                    
                    if ( abs(*_vibOutput) < 0.0001f)
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

bool vibrator::run(ros::Time myCurrentTime)
{
    update(myCurrentTime);
    return _flagTrigger;
}

void vibrator::start()
{
    if(!_flagTrigger)
    {
        _flagTrigger = true;
        _flagReset=false;
    }
}

void vibrator::reset(){

    if(!_flagReset && _myStatus==FINISHED)
    {
       _myStatus=STANDBY;
       _flagTrigger=false;
       _flagReset=true;
    }
}