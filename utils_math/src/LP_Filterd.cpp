#include "LP_Filterd.h"

LP_Filterd::LP_Filterd()
{
  _output = 0.0f;
  _old_output = 0.0f;
  _alpha = 0.0f;
  _bias = 0.0f;
}

LP_Filterd::LP_Filterd(double alpha)
{
  _output=0.0f;
  _old_output=0.0f;
  _alpha=alpha;
  _bias=0.0f;
}


double LP_Filterd::update(double raw_input){
  _output=_alpha*_old_output + (1.0f-_alpha)*raw_input;
  _old_output=_output;
  return _output - _bias;
}

void LP_Filterd::setBias(double bias_)
{
  _bias=bias_;
}

void LP_Filterd::reset()
{
  _output=0.0f;
  _old_output=0.0f;
}

void LP_Filterd::setAlpha(double alpha)
{
  _alpha=alpha;
}