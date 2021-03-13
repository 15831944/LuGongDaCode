#ifndef DATA_SOURCE_H
#define DATA_SOURCE_H
#include <common/Types.hpp>
#include <dsp/PrbsGenerator.h>
#include <dsp/SSE.h>

class data_source
{
private:
	BYTE *mData;
	Uint4 mSynLen;
	Uint4 mDataLen;
	Uint4 mT;
	Uint4 oPos;
	Uint4 HaveData;
	prbs_generator *Prbs;
public:
	data_source(Uint4 SynLen,Uint4 DataLen);
	~data_source(); 
	void setSyn(BYTE *Syn,Uint4 SynLen);
	void outputData(BYTE *output, Uint4 outLen);
	void generateData();
};

#endif
