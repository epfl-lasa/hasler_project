#include <lp_filter.h>

lp_filter::lp_filter(float alpha)
{
  _output=0.0f;
  _old_output=0.0f;
  _alpha=alpha;
}

float lp_filter::update(float raw_input){
  _output=_alpha*_old_output + (1.0-_alpha)*raw_input;
  _old_output=_output;
  return _output;
}

void lp_filter::reset()
{
  _output=0.0f;
  _old_output=0.0f;
}

// void lp_filter::setValue(float alpha)
// {
//   _alpha=alpha;
// }