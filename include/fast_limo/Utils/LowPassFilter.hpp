
#ifndef _FAST_LIMO_LPF_HPP_
#define _FAST_LIMO_LPF_HPP_

#include <cmath>

#define ERROR_CHECK (true)

#if ERROR_CHECK
#include <iostream>
#endif

class LowPassFilter{
	public:
			//constructors
		LowPassFilter();
		LowPassFilter(float iCutOffFrequency, float iDeltaTime);
			//functions
		float update(float input);
		float update(float input, float deltaTime, float cutoffFrequency);
			//get and configure funtions
		float getOutput() const {return output;}
		void reconfigureFilter(float deltaTime, float cutoffFrequency);
	private:
		float output;
		float ePow;
};

LowPassFilter::LowPassFilter():
	output(0),
	ePow(0){}

LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime):
	output(0),
	ePow(1-exp(-iDeltaTime * 2 * M_PI * iCutOffFrequency))
{
	#if ERROR_CHECK
	if (iDeltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	if(iCutOffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow = 0;
	}
	#endif
}

float LowPassFilter::update(float input){
	return output += (input - output) * ePow;
}

float LowPassFilter::update(float input, float deltaTime, float cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); // changes ePow accordingly.
	return output += (input - output) * ePow;
}

void LowPassFilter::reconfigureFilter(float deltaTime, float cutoffFrequency){
	#if ERROR_CHECK
	if (deltaTime <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	if(cutoffFrequency <= 0){
		std::cout << "Warning: A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow = 0;
	}
	#endif
	ePow = 1-exp(-deltaTime * 2 * M_PI * cutoffFrequency);
}

#endif //_FAST_LIMO_LPF_HPP_
