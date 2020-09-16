#ifndef __LP_FILTER__
#define __LP_FILTER__

class LP_Filter
{
	public:
		LP_Filter(); 
		LP_Filter(float alpha);
		float update(float raw_input);
		void setAlpha(float alpha);		
		void reset();
		void setBias(float bias_);
	private:
		float _bias;
		float _alpha;
		float _old_output;
		float _output;
};

#endif /*__LP_FILTER__*/
