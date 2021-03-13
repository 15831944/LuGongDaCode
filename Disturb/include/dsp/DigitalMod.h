#ifndef DIGITAL_MOD_H
#define DIGITAL_MOD_H

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>
#include <string.h>  
#include <vector>
#include <assert.h>

#include <common/Types.hpp>
#include <common/Complex.hpp>
#include <dsp/SSE.h>
#include <dsp/FirDesign.h>
#include <dsp/Filter.h>
#include <dsp/DigitalSig.h>

//
class DigitalMod
{
protected:
	DigMOD mMod;
	PULSE mPulse;
	Uint4 mSPS; //samples per symbol
	Uint4 mTS; //duration of one symbol
	Uint4 mBPS;//bits per symbol
	Uint4 mRadix;
	filter_cf *firGT;
	fir_design firdes;
	fcomplex Mapper[128]; //the bigest is 128QAM
	Uint4	  gcode[128];
	std::vector<float> mTaps;
  
	
//FOR FSK	
	float   mPhase;
	float   mDeltaW;
	Uint4    mPLS; //duration samples of one packet(SYN+DATA); 
	Uint4    Pcounter;  
public:
	DigitalMod(DigPram Pram);
	~DigitalMod();
	Uint4 BPS(){std::cout<<"mBPS="<<mBPS<<std::endl;return mBPS;}
	Uint4 SPS(){return mSPS;}
	std::vector<float> taps(){return mTaps;}
	filter_cf *getGT(){return firGT;}
	void initMapper();
	void reset();
	void setDeltaW(float deltaW){mDeltaW=deltaW;}
	void setInitPhase(float Phase){mPhase=Phase;}
	//(inLen % mBPS) 必须是 0， 否则将会有比特丢失，程序中不会提示
	Uint4 Modulate(BYTE *data, fcomplex *output, Uint4 inLen, Uint4 delay);
	Uint4 Modulate(BYTE *data, complex16 *output, Uint4 inLen, Uint4 delay,float amp);
	std::vector<fcomplex> ModulateGTGR(BYTE *data, Uint4 inLen);
	Uint4 ModulateLinear(BYTE *data, fcomplex *output, Uint4 inLen,Uint4 delay);//线性调制
	Uint4 ModulateFSK(BYTE *data, fcomplex *output, Uint4 inLen,Uint4 delay);//FSK调制,非线性
	
};

class FSKMod:public DigitalMod
{
protected:
    DigitalMod *dMod;
public:
	FSKMod();
	~FSKMod();		
};



#endif
