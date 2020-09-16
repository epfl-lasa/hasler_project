#ifndef __LP_FILTERD__
#define __LP_FILTERD__

class LP_Filterd
{
	public:
		LP_Filterd(); 
		LP_Filterd(double alpha);
		double update(double raw_input);
		void setAlpha(double alpha);		
		void reset();
		void setBias(double bias_);
	private:
		double _bias;
		double _alpha;
		double _old_output;
		double _output;
};

#endif /*__LP_FILTERD__*/
