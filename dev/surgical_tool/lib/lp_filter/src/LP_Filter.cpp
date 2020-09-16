#include <LP_Filter.h>

LP_Filter::LP_Filter()
{
  _output = 0.0f;
  _old_output = 0.0f;
  _alpha = 0.0f;
  _bias = 0.0f;
}

LP_Filter::LP_Filter(float alpha)
{
  _output=0.0f;
  _old_output=0.0f;
  _alpha=alpha;
  _bias=0.0f;
}


float LP_Filter::update(float raw_input){
  _output=_alpha*_old_output + (1.0f-_alpha)*raw_input;
  _old_output=_output;
  return _output - _bias;
}

void LP_Filter::setBias(float bias_)
{
  _bias=bias_;
}

void LP_Filter::reset()
{
  _output=0.0f;
  _old_output=0.0f;
}

void LP_Filter::setAlpha(float alpha)
{
  _alpha=alpha;
}