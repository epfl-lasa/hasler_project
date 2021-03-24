#ifndef __GAUSSIANFILTER__
#define __GAUSSIANFILTER__


#include <cmath>
#include "Utils_math.h"
#define NB_LIMS 2
#define L_MIN 0
#define L_MAX 1

template<typename T>
class gaussianFilter
{
	public:
		gaussianFilter(); 
		gaussianFilter(T sigma, T x0);
		T update(T raw_input);
		void setOutputLimits(T minLim, T maxLim);
		void setSigma(T sigma);		
		void setX0(T x0);		
		void reset(T raw_input);
		void setBias(T bias_);
	private:
		T _bias;
		T _sigma;
		T _x0;
		T _output;
		T _lims[NB_LIMS];
};

#endif /*__GAUSSIANFILTER__*/
