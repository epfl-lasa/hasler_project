#include <MA_Filter.h>

MA_Filter::MA_Filter( int sampleSize):
_nSample(sampleSize)
{
  _output=0.0f;
  _inputs=0.0f;
  _iSample=0.0f;
  _bias=0.0f;
}

float MA_Filter::update(float raw_input){
  if (_iSample<_nSample)
  {
    _inputs+=raw_input;
    _iSample++;
  }
  else
  {
    _output=_inputs/_nSample;
    _output = _output - _bias;
    _iSample = 0;
    _inputs = 0.0f;
  }

  return _output;
}

void MA_Filter::reset()
{
  _output=0.0f;
  _inputs=0.0f;
  _bias=0.0f;
}
