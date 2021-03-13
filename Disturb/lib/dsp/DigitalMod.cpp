#include <iostream>

#include <common/GNUPlot.h>
#include <dsp/DigitalMod.h>
#include <dsp/SigProcLib.h>  
using namespace std;
extern GNUPlot ploter;
float PhaseTPI;

char ModStr[5][16]={"ASK","BPSK","FSK","QPSK","PSK8"};

char PulseStr[3][32]={"平方根升余弦","升余弦","矩形"};

char PrbsStr[8][8]={"PRBS4","PRBS5","PRBS6","PRBS9","PRBS11","FILE1","FILE2","FILE3"};

DigitalMod::DigitalMod(DigPram Pram)
:mMod(Pram.Mod),mPulse(Pram.Pulse),mSPS(Pram.SPS),mTS(Pram.TS)
{
	assert(Pram.SynBits>0);
	assert(Pram.DataBits>0);
	
	firGT= new filter_cf(mTS);		  
	switch(mPulse){
		case RT_RCOSINE:mTaps=firdes.root_raised_cosine(1,mSPS,1,Pram.alpha,mTS);break;
		case SQUARE:mTaps.assign(mTS,1.0/mTS);break;
		default:;break;
	}

	firGT->setTaps(mTaps);
	
	initMapper();//generate mapper and get mBPS, mRAdix;
	
	assert(mBPS>0);
	mPLS=(Pram.SynBits+Pram.DataBits)*mSPS/mBPS;	
	PhaseTPI=1000*M_PI;
	mPhase=0;		
	Pcounter=0;	
	//cout<<"create ok mBPS="<<mBPS<<" "<<BPS()<<endl;
	
	if(Pram.Mod==FSK){
		assert(Pram.FSKRate>0);
		setDeltaW(Pram.FSKRate);
	}
}

DigitalMod::~DigitalMod()
{
	delete (filter_cf *) firGT;
}


/* exmaple of 8 
 * B		G
0: 0 0 0  | 0 0 0 ==>0
1: 0 0 1  | 0 0 1 ==>1
2: 0 1 0  | 0 1 1 ==>3
3: 0 1 1  | 0 1 0 ==>2
4: 1 0 0  | 1 1 0 ==>6
5: 1 0 1  | 1 1 1 ==>7
6: 1 1 0  | 1 0 1 ==>5
7: 1 1 1  | 1 0 0 ==>4
*/
Uint4 ModtoBPS(DigMOD Mod)
{
	Uint4 BPS=0;
	switch(Mod){
		case ASK: BPS=1;break;
		case BPSK:BPS=1;break;
		case QPSK: BPS=2;break;
		case PSK8: BPS=3;break;
		case FSK: BPS=1;break;
		default: BPS=0;break;
	}
	return BPS;
	
}

void showDigPram(DigPram dPram)
{
	cout<<"=============Parameters of DigPram============="<<endl;
	cout<<"  Moulation=   "<<ModStr[dPram.Mod]<<endl; 
	cout<<"  Pulse=       "<<PulseStr[dPram.Pulse]<<endl;
	cout<<"  Ts=          "<<dPram.TS<<endl; 
	cout<<"  alpha=       "<<dPram.alpha<<endl<<endl;
	
	cout<<"  SPS=         "<<dPram.SPS<<endl;
	cout<<"  Rb=          "<<dPram.Rb<<endl;
	cout<<"  Rs=          "<<dPram.Rs<<endl;
	cout<<"  Fs=          "<<dPram.Fs<<endl;
	
	cout<<"  FSKRate=     "<<dPram.FSKRate<<endl<<endl;
	
	cout<<"  SynBits=     "<<dPram.SynBits<<endl;
	cout<<"  DataBits=    "<<dPram.DataBits<<endl;
	
	cout<<"  SynFormat=   "<<PrbsStr[dPram.SynFormat]<<endl;
	cout<<"  DataFormat=  "<<PrbsStr[dPram.DataFormat]<<endl;
	cout<<"================================================"<<endl;
    cout<<endl;
}

void DigitalMod::initMapper()
{
	switch(mMod){
		case ASK: 
			Mapper[0]=fcomplex(0,0); 
			Mapper[1]=fcomplex(1,0);
			mBPS=1;
			break;
		case BPSK:
			mBPS=1;
			mRadix=(1<<mBPS);
			for(Uint4 i=0;i<mRadix;i++) {Uint4 index=graycode(i,mBPS); gcode[index]=i;}
			for(Uint4 i=0;i<mRadix;i++) Mapper[i]=expj(gcode[i]*M_PI*2/mRadix);
			break;
		case QPSK:
			mBPS=2;
			mRadix=(1<<mBPS);
			for(Uint4 i=0;i<mRadix;i++) {Uint4 index=graycode(i,mBPS); gcode[index]=i;}
			for(Uint4 i=0;i<mRadix;i++) Mapper[i]=expj(gcode[i]*M_PI*2/mRadix);
			break;
		case PSK8:
			mBPS=3;
			mRadix=(1<<mBPS);
			for(Uint4 i=0;i<mRadix;i++) {Uint4 index=graycode(i,mBPS); gcode[index]=i;}
			for(Uint4 i=0;i<mRadix;i++) Mapper[i]=expj(gcode[i]*M_PI*2/mRadix);			
			break;
		case FSK:
			mBPS=1;
			mRadix=(1<<mBPS);
			Mapper[0]=fcomplex(1,-1);
			Mapper[1]=fcomplex(1,1);
		default:mBPS=1;break;
	}
}

std::vector<fcomplex> DigitalMod::ModulateGTGR(BYTE *data, Uint4 inLen)
{
	reset();
	Uint4 outSymbols=inLen/mBPS;
	Uint4 outLen=outSymbols*mSPS;
	
	fcomplex *sigmod= new fcomplex[outLen+mTS];
	fcomplex *sigfir= new fcomplex[outLen+mTS];
	
	bzero(sigmod,(outLen+mTS)*sizeof(fcomplex));
	bzero(sigfir,(outLen+mTS)*sizeof(fcomplex));
	
	Uint4 offset;
	if((mTS % 2)==0) offset=1; else offset=0;
	Uint4 delay=mTS-mSPS+offset;
	Modulate(data,sigmod,inLen,delay);
	

	// GT=GR

	firGT->filter(sigmod,sigfir,outLen+delay);
	
	std::vector<fcomplex> outVec;
	
	for(Uint4 i=0;i<outLen;i++) outVec.push_back(sigfir[i+delay]);
	
	
	delete (fcomplex *) sigmod;
	delete (fcomplex *) sigfir;
	
	return outVec;
}

Uint4 DigitalMod::Modulate(BYTE *data, fcomplex *output, Uint4 inLen,Uint4 delay)
{
	switch(mMod){
		case ASK:return ModulateLinear(data, output,inLen,delay);break;
		case BPSK:return ModulateLinear(data, output,inLen,delay);break;
		case QPSK:return ModulateLinear(data, output,inLen,delay);break;
		case PSK8:return ModulateLinear(data, output,inLen,delay);break;
		case FSK:return ModulateFSK(data, output,inLen,delay);break;
	}
}

Uint4 DigitalMod::Modulate(BYTE *data, complex16 *output, Uint4 inLen,Uint4 delay, float amp)
{
	fcomplex *toutput=new fcomplex[2000];
	int outlen=0;
	switch(mMod){
		case ASK:outlen=ModulateLinear(data, toutput,inLen,delay);break;
		case BPSK:outlen=ModulateLinear(data, toutput,inLen,delay);break;
		case QPSK:outlen=ModulateLinear(data, toutput,inLen,delay);break;
		case PSK8:outlen=ModulateLinear(data, toutput,inLen,delay);break;
		case FSK:outlen=ModulateFSK(data, toutput,inLen,delay);break;
	}
	for(Uint4 i=0;i<outlen;i++){
		output[i]=fcomplex(toutput[i].real()*amp,toutput[i].imag()*amp);
	}
	delete toutput;
	return outlen;
}
/*
 *  data   +++++++++++  sigPulse  ++++++++++   output
 * ======> + Mapper  + =========> +  firGT + =========> 
 *         +++++++++++            ++++++++++
 */
 //inLen is the bits' num input
Uint4 DigitalMod::ModulateLinear(BYTE *data, fcomplex *output, Uint4 inLen,Uint4 delay)
{
	Uint4 outSymbols=inLen/mBPS;
	Uint4 outLen=outSymbols*mSPS+delay;
	
	fcomplex *sigPulse= new fcomplex [outLen];
	
	bzero(sigPulse,outLen*sizeof(fcomplex));	
	
	for(Uint4 i=0;i<outSymbols;i++){
		Uint4 starData=0;
		for(Uint4 j=0;j<mBPS;j++){
			//cout<<(int)data[i*mBPS+j];
			starData=(starData<<1)+(data[i*mBPS+j] & 0x1);
		}
		sigPulse[mSPS*i]=Mapper[starData];
		//cout<<" starData="<<starData<<" "<<sigPulse[mSPS*i]<<endl;
	}

	firGT->filter(sigPulse,output,outLen);
		
	delete (fcomplex*) sigPulse;	
	return outLen;
	
}



void DigitalMod::reset()
{
	firGT->clear();
}




/*
 *  data   +++++++++++  sigPulse  ++++++++++   output
 * ======> + Mapper  + =========> +  firGT + =========> 
 *         +++++++++++            ++++++++++
 */
 //inLen is the bits' num input
 
Uint4 DigitalMod::ModulateFSK(BYTE *data, fcomplex *output, Uint4 inLen,Uint4 delay)
{ 
	Uint4 outSymbols=inLen/mBPS;
	Uint4 outLen=outSymbols*mSPS+delay;
	
	fcomplex *sigPulse= new fcomplex [outLen];
	
	bzero(sigPulse,outLen*sizeof(fcomplex));	
	
	for(Uint4 i=0;i<outSymbols;i++){
		Uint4 starData=0;
		for(Uint4 j=0;j<mBPS;j++){
			//cout<<(int)data[i*mBPS+j];
			starData=(starData<<1)+(data[i*mBPS+j] & 0x1);
		}
		sigPulse[mSPS*i]=Mapper[starData];
		//cout<<" starData="<<starData<<" "<<sigPulse[mSPS*i]<<endl;
	}

	firGT->filter(sigPulse,output,outLen);
	float In,Qn;
	float deltaP=2.0*M_PI*mDeltaW/float(mSPS);
	//cout<<"deltaP="<<deltaP<<" M_PI="<<M_PI<<" deltaW="<<deltaW<<endl;
	for(Uint4 i=0;i<outLen;i++){
		In=output[i].real();
		Qn=output[i].imag();
		In=In*cosLookup(mPhase);  
		Qn=Qn*sinLookup(mPhase);  
		mPhase=mPhase+deltaP;	  
		//cout<<Phase<<" ";	
		output[i]=fcomplex(In,Qn);
		Pcounter++;
		if(Pcounter>=mPLS){Pcounter=0;mPhase=0;}//相位在没一包的开始，需要请零
	}
	if(mPhase>PhaseTPI) mPhase=mPhase-PhaseTPI;
	//cout<<endl;
		
	delete (fcomplex*) sigPulse;	
	return outLen;
	
}

