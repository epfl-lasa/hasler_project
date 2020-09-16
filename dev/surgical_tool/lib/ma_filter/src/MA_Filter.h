#ifndef __MA_FILTER__
#define __MA_FILTER__

class MA_Filter
{
	public:
		MA_Filter(int sampleSize); 
		float update(float raw_input);
		void reset();
		float _bias;

	private:
		float _inputs;
		float _output;
		int _nSample;
		int _iSample;
		
};

#endif /*__MA_FILTER__*/
