#ifndef __HP_FILTERD__
#define __HP_FILTERD__
#include <iostream>

#include "math.h"
template<typename T = float>
class HP_Filterd
{
	public:
		HP_Filterd(); 
		HP_Filterd(T alpha);
		T update(T raw_input);
		void setInit(T oldOutput, T oldInput);
		void setAlpha(T alpha);
		void setParameters(T freq, T dt);
		T getAlpha();		
		void reset();
		void setBias(T bias_);
		T getOutput();
	private:
		bool _firstInputReceived;
		T _bias;
		T _alpha;
		T _dt;
		T _old_output;
		T _old_input;
		T _output;
};

#endif /*__HP_FILTERD__*/
