/* 
 * This Program can generate prbs data sequence / m sequence
 * More information please refer to
 * RS_SMU200A_Operating.pdf
 * 4.8.2.1
 * Internal PRBS Data and Data Patterns
 * 
 */
#ifndef PRBS_GENERATOR_H
#define PRBS_GENERATOR_H
#include <string.h> 
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <common/Types.hpp>
#include <dsp/SSE.h>
#include <dsp/DigitalSig.h>

class prbs_generator 
{
protected:
	PRBS 			mStyle;
	BYTE 			*mRegs;
	BYTE 			*mTaps;
	Uint4	 		mRegLen;
	Uint4  			mT;
public:
	prbs_generator(PRBS mStyle);
	~prbs_generator();
	Uint4 T(){return mT;}
	
	void reset();
	void generate(BYTE *output, Uint4 Len);
	std::vector <BYTE> generate(Uint4 Len);
	
};
std::vector<int> strtoIntVector(std::string str);
#endif
