
#include <iostream>
#include <vector>
#include <string.h>
#include <math.h>

#include <dsp/PrbsGenerator.h>
using namespace std;

std::vector<int> strtoIntVector(std::string str)
{
	std::vector<int> retVal;
	if(str[0]>0){
		char *lp=&str[0];
		while (lp) {
			retVal.push_back(strtol(lp,NULL,10));
			strsep(&lp," ");
		}
	}
	return retVal;
}

prbs_generator::prbs_generator(PRBS Style)
{
	mStyle=Style;
	std::vector<int> fbp;
	switch(Style){
		case PRBS4: mRegLen=4; fbp=strtoIntVector("1 0");break;
		case PRBS5: mRegLen=5; fbp=strtoIntVector("2 0");break;
		case PRBS6: mRegLen=6; fbp=strtoIntVector("1 0");break;
		case PRBS9: mRegLen=9; fbp=strtoIntVector("4 0"); break;
		case PRBS11: mRegLen=11;  fbp=strtoIntVector("2 0"); break;
		//case PRBS16: mRegLen=16;  fbp=strtoIntVector("5 3 2 0"); break;
		//case PRBS20: mRegLen=20;  fbp=strtoIntVector("3 0"); break;
		//case PRBS21: mRegLen=21;  fbp=strtoIntVector("2 0"); break;
	}
	mRegs=new BYTE [mRegLen];
	mTaps= new BYTE [mRegLen];
	bzero(mTaps,mRegLen);
	bzero(mRegs,mRegLen);
	mRegs[0]=1;
	for(Uint4 i=0;i<fbp.size();i++) mTaps[fbp[i]]=1;
	mT=(1<<mRegLen)-1;
	
}

prbs_generator::~prbs_generator()
{
	delete (BYTE *) mRegs;
	delete (BYTE *) mTaps;
	
}
void prbs_generator::reset()
{
	bzero(mRegs,mRegLen);
	mRegs[0]=1;
}
std::vector <BYTE> prbs_generator::generate(Uint4 Len)
{
	std::vector <BYTE> bits;
	for(Uint4 i=0;i<Len;i++){
		bits.push_back(mRegs[0]);
		BYTE feedback=0;
		for(Uint4 j=0;j<mRegLen;j++){
			feedback=feedback^(mRegs[j] & mTaps[j]);;
		}
		for(Uint4 j=0;j<mRegLen-1;j++){
			mRegs[j]=mRegs[j+1];
		}
		mRegs[mRegLen-1]=feedback;
		
	}
	return bits;
	
}
void prbs_generator::generate(BYTE *output, Uint4 Len)
{
	for(Uint4 i=0;i<Len;i++){
		output[i]=mRegs[0];
		BYTE feedback=0;
		for(Uint4 j=0;j<mRegLen;j++){
			feedback=feedback^(mRegs[j] & mTaps[j]);;
		}
		for(Uint4 j=0;j<mRegLen-1;j++){
			mRegs[j]=mRegs[j+1];
		}
		mRegs[mRegLen-1]=feedback;
		
	}
}

