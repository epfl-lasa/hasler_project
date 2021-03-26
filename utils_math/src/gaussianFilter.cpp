#include "gaussianFilter.h"

template<typename T>
gaussianFilter<T>::gaussianFilter()
{
  _output = 0.0;
  _x0 = 0.0;
  _sigma = __FLT_EPSILON__;
  _bias = 0.0;
  _lims[L_MIN]= 0.0;
  _lims[L_MAX]= 0.0;
}

template<typename T>
gaussianFilter<T>::gaussianFilter(T sigma, T x0):
_sigma(sigma),_x0(x0)
{
  _output=0.0;
  _bias=0.0;
  _lims[L_MIN]=0.0;
  _lims[L_MAX]=0.0;
}


template<typename T>
void gaussianFilter<T>::setOutputLimits(T minLim, T maxLim)
{
  _lims[L_MIN] = minLim;
  _lims[L_MAX] = maxLim;
}

template<typename T>
T gaussianFilter<T>::update(T raw_input){
  
  _output = Utils_math<T>::bound(_bias + ( L_MAX - _bias ) * exp(-0.5 * pow((raw_input-_x0),2.0) * (1.0f/pow(_sigma,2))),L_MIN,L_MAX);
  return _output;

}

template<typename T>
void gaussianFilter<T>::setBias(T bias_)
{
  _bias=bias_;
}

template<typename T>
void gaussianFilter<T>::reset(T raw_input)
{
  _output= raw_input;
}

template<typename T>
void gaussianFilter<T>::setSigma(T sigma)
{
  _sigma=sigma;
}


template<typename T>
void gaussianFilter<T>::setX0(T x0)
{
  _x0=x0;
}


template class gaussianFilter<float>;
template class gaussianFilter<double>;