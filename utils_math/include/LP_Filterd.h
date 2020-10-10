#ifndef __LP_FILTERD__
#define __LP_FILTERD__
#include <iostream>

#include "math.h"

class LP_Filterd
{
	public:
		LP_Filterd(); 
		LP_Filterd(double alpha);
		double update(double raw_input);
		void setAlpha(double alpha);		
		double getAlpha();		
		void reset();
		void setBias(double bias_);
		double getOutput();
	private:
		double _bias;
		double _alpha;
		double _old_output;
		double _output;
};

#endif /*__LP_FILTERD__*/
