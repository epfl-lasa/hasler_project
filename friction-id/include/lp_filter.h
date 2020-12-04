#ifndef __LP_FILTER__
#define __LP_FILTER__

class lp_filter
{
	public:
		lp_filter(float alpha); 
		float update(float raw_input);		
		void reset();
	private:
		float _alpha;
		float _old_output;
		float _output;
};

#endif /*__LP_FILTER__*/
