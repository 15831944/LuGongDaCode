#include <iostream>
#include <assert.h>
#include <string.h>
#include <dsp/DataSource.h>
#include <dsp/PrbsGenerator.h>
#include <dsp/SigProcLib.h>

using namespace std;
void data_source::setSyn(BYTE *Syn,Uint4 SynLen)
{
	assert(SynLen<=mSynLen);
	memcpy(mData,Syn,SynLen);
}      

void data_source::outputData(BYTE *output, Uint4 outLen)
{
	Uint4 readLen,readLen2;
	assert(outLen<=mT);//一次输出的长度不能大于一个数据周期
	if(HaveData>0){
		if(HaveData>=outLen){
			readLen=outLen;
			memcpy(output,mData+oPos,readLen);
			HaveData=HaveData-readLen;
			oPos=oPos+readLen;
		}else{
			readLen=HaveData;
			memcpy(output,mData+oPos,readLen);
						
			generateData();			
			readLen2=outLen-readLen;
			memcpy(output+readLen,mData+oPos,readLen2);
			HaveData=HaveData-readLen2;
			oPos=oPos+readLen2;		
		
		}
	}else{
		generateData();
		readLen=outLen;
		memcpy(output,mData+oPos,readLen);
		HaveData=HaveData-readLen;
		oPos=oPos+readLen;	
	}

	
}
void data_source::generateData()
{

/*	for(Uint4 i=mSynLen;i<mT;i++) {
		mData[i]=(random() & 1);
		//mData[i]=i&0x1;
	}*/
	Prbs->generate(mData+mSynLen,(mT-mSynLen));
	oPos=0;
	HaveData=mT;
	
}

data_source::data_source(Uint4 SynLen, Uint4 DataLen)
{
	mSynLen=SynLen;
	mDataLen=DataLen;
	mT=(mSynLen+mDataLen);
	mData=new BYTE[mT];
	oPos=0;	
	HaveData=0;
	Prbs= new prbs_generator(PRBS9);
}




data_source::~data_source( )
{
	delete (BYTE *) mData;
	delete (prbs_generator *) Prbs;
}
