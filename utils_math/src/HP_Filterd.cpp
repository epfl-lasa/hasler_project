#include "HP_Filterd.h"

template<typename T>
HP_Filterd<T>::HP_Filterd()
{
  _output = 0.0f;
  _old_output = 0.0f;
  _firstInputReceived = false;
  _alpha = 0.0f;
  _bias = 0.0f;
}
template<typename T>
HP_Filterd<T>::HP_Filterd(T alpha)
{
  _output=0.0f;
  _old_output=0.0f;
  _firstInputReceived = false;
  _alpha=alpha;
  _bias=0.0f;
}

template<typename T>
T HP_Filterd<T>::update(T raw_input){
  if (!isnan(raw_input))
  { 
    if (!_firstInputReceived)
    {
      _old_input = raw_input;
      _output = 0.0f;
      _firstInputReceived=true;
    }
    else
    {
      _output= (_alpha) * (_old_output  + raw_input - _old_input);  
    }
  }
  _old_input = raw_input;
  _old_output=_output;
  return _output - _bias;
}

template<typename T>
void HP_Filterd<T>::setBias(T bias_)
{
  _bias=bias_;
}

template<typename T>
void HP_Filterd<T>::reset()
{
  _output=0.0f;
  _old_output=0.0f;
  _old_input=0.0f;
}

template<typename T>
T HP_Filterd<T>::getOutput()
{
  return _output;
}

template<typename T>
T HP_Filterd<T>::getAlpha()
{
  return _alpha;
}

template<typename T>
void HP_Filterd<T>::setAlpha(T alpha)
{
  _alpha=alpha;
}

template<typename T>
void HP_Filterd<T>::setParameters(T freq, T dt)
{
  _alpha=1.0 / (2*M_PI * dt * freq + 1);
}

template<typename T>
void HP_Filterd<T>::setInit(T oldOutput, T oldInput)
{
  _old_output=oldOutput;
  _old_input = oldInput;
}

template class HP_Filterd<float>;
template class HP_Filterd<double>;
