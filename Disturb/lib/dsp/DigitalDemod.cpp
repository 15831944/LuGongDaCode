#include <iostream>
#include <dsp/DigitalDemod.h>
#include <dsp/SigProcLib.h>
extern GNUPlot ploter;    
using namespace std;
   
SynDetect::SynDetect(Uint4 FL, Uint4 DL,Uint4 SPS)
:mFL(FL),mDL(DL),mSPS(SPS)
{

	if((mFL%2)!=0) FL2=mFL+1;else FL2=mFL;
	mTaps= (fcomplex *) malloc_with_16bytes_alignment(FL2*sizeof(fcomplex));
	mem =(fcomplex *)  malloc_with_16bytes_alignment(FL2*sizeof(fcomplex));
	add_sse=(float *) malloc_with_16bytes_alignment(4*sizeof(float));
	bzero(mTaps,FL2*sizeof(fcomplex));
	bzero(mem,FL2*sizeof(fcomplex));
	
	convbuf=new fcomplex[256];//便于循环移位
	
	mT=mFL+mDL;
	state=EN_DETECT;
	avpower=0;
	pos=0;		
	dtheta=0;
	ctheta=0;
	Lost_SYN_Num=0;
	Rec_Samples=0;
	convbufpos=0;
	toaestimate=false;
	toa=0;
}

SynDetect::~SynDetect()
{
	free_with_16bytes_alignment(mTaps);
	free_with_16bytes_alignment(mem);
	free_with_16bytes_alignment(add_sse);
	delete convbuf;
}

void SynDetect::setTaps(std::vector<fcomplex> taps)
{
	assert(taps.size()==mFL);
	TapEN=0;
	for(Uint4 i=0;i<mFL;i++) { 
		mTaps[i]=taps[mFL-i-1].conj();//这里必须反转一下滤波器.并且共轭
		TapEN=TapEN+mTaps[i].norm2();
	}
	assert(TapEN>0);
	
}
std::vector<SynInfo>
SynDetect::work_test(fcomplex *input,float *output, int Len)
{
	std::vector<SynInfo> outVec;
	fcomplex conv;
	float peak;
	//avpower=10000;
	for(int i=0;i<Len;i++){
		conv=convshift(input[i], &avpower);	
		peak=conv.norm2();///avpower;
		output[i]=peak;
	}			
	//cout<<avpower<<endl;
	return outVec;
	
}

  
std::vector<SynInfo>
SynDetect::work(fcomplex *input,fcomplex *output, int Len)
{
	fcomplex conv;
	float peak;
	Rec_Samples=Rec_Samples+Len;
	if(Rec_Samples>=mT) {
		Lost_SYN_Num++;
		Rec_Samples=Rec_Samples % mT;
		//cout<<"Lost_SYN_Num="<<Lost_SYN_Num<<endl;
	}
	std::vector<SynInfo> outVec;
	for(int i=0;i<Len;i++){
		switch(state){
			case EN_DETECT:
			{
				estimateEnergy(input[i],&avpower);
				if(avpower>mPTH){
					state=SYN_NO;
				}
				output[i]=0;
			//	cout<<avpower<<" "<<mPTH<<endl;
				//output[i]=fcomplex(output[i].real(),avpower);				
			}break;//case EN_DETECT
			
			case SYN_NO:
			{
				conv=convshift(input[i], &avpower);	
				
				convbuf[convbufpos]=conv;
				convbufpos=convbufpos+1;
				
				peak=conv.norm2()/avpower;
				output[i]=fcomplex(peak,0);
				//output[i]=conv.norm2();
				if(peak>mCTH){
					state=SYN_MAY;
					needwatch=mSPS-1;
					maxpeak=peak;
					pos=i;	
					posPeakbuf=convbufpos-1;	
					toaestimate=true;
					toa=0;	
					if(needwatch==0) //if SPS=1 
					{
						
						float ampchan=conv.abs()/mFL;
						chan=(conv.conj()/conv.abs())/ampchan*mADJ;	
					//cout<<"chan="<<chan<<endl;			
						state=SYN_OK;	
						begin=true;
						Lost_SYN_Num=0;
						counter=0;
						needwait=mSPS-(i-pos);	
					//	cout<<"needwait="<<needwait<<endl;
					  //  cout<<endl<<"SYN_NO peak="<<maxpeak<<" avpower="<<avpower<<endl;	
					    deltaf=estimateFreq(mSPS-needwait+1); //频率误差校准
					   //  cout<<"avpower="<<avpower<<" peak="<<peak<<" deltaf="<<deltaf*200e3<<endl;
					  //  cout<<"syn no estimate deltaf="<<deltaf*200e3<<endl;
					}
				}
				 
			}break;//case SYN_NO
			
			
			case SYN_MAY:
			{
				conv=convshift(input[i], &avpower);	
				convbuf[convbufpos]=conv;
				convbufpos=convbufpos+1;
				peak=conv.norm2()/avpower;
				output[i]=fcomplex(peak,0);
				//output[i]=conv.norm2();
				if(peak>maxpeak){
					pos=i; maxpeak=peak;
					posPeakbuf=convbufpos-1;
				}
				needwatch--;
				if(needwatch==0){
					
					
					float ampchan=conv.abs()/mFL;
					//cout<<"ampchan="<<ampchan<<endl;
					chan=(conv.conj()/conv.abs())/ampchan*mADJ;	
					//cout<<"chan="<<chan<<endl;			
					state=SYN_OK;	
					begin=true;
					Lost_SYN_Num=0;
					counter=0;
					needwait=mSPS-(i-pos);
					//cout<<"need wait="<<needwait<<endl;	
				//	cout<<endl<<"SYN_MAY peak="<<maxpeak<<" avpower="<<avpower<<endl;	
					 deltaf=estimateFreq(mSPS-needwait+1); //频率误差校准
					 
					//  cout<<"may avpower="<<avpower<<" peak="<<peak<<" deltaf="<<deltaf*200e3<<endl;
					// deltaf=-97.6562/200e3;
					// deltaf=0;
					//cout<<"syn may estimate deltaf="<<deltaf*200e3<<endl;
					
				}		
			}break;//case SYN_MAY
			
			case SYN_OK:
			{                 //4 mSPS
				if (counter<=(mSPS<<2)) //找到相关峰值后，继续进行一部分相关运算，主要是为了微调采样位置使用
				{
					conv=convshift(input[i], &avpower);	
					convbuf[convbufpos]=conv;
					convbufpos=convbufpos+1;
				}
				else
				{
					if (toaestimate){
						//ploter.plotabs(convbuf,256);
						//getchar();
						toaestimate=false;
						//cout<<"maxpos="<<posPeakbuf<<endl;
						unsigned char eP,aP;
						eP=posPeakbuf-1;
						aP=posPeakbuf+1;
						float a, e ,m;
						a=convbuf[aP].abs();
						e=convbuf[eP].abs();
						m=convbuf[posPeakbuf].abs();
						toa=-0.5*(a-e)/(a+e-2*m);
						cout<<"eP="<<e<<" max="<<m<<" aP="<<a<<" toa="<<toa<<endl;
					}
					
					estimateEnergy(input[i],&avpower);
				}

				//cout<<avpower<<" ";
				//output[i]=fcomplex(0,avpower);
				needwait--;
				ctheta=ctheta+dtheta;
				if(needwait<=0){
					if(counter<=mDL-mSPS){
						outVec.push_back(SynInfo {i,chan,begin, deltaf});
						begin=false;
						
						output[i]=fcomplex(0,1);
					}					
					counter=counter+mSPS;
					if(counter>=mDL){
						//clear();
						state=SYN_NO;
					}
					needwait=mSPS;
					//cout<<i<<" "<<endl;
				}
				
				
			}break;//case SYN_OK;
			default:break;
		}			
	}	
	pos=pos-Len; //当出现 SYN_NO在上一包，而SYN_MAY 在下一包时非常关键
	return outVec;


}

void SynDetect::clear()
{
	state=EN_DETECT;
	avpower=0;
	pos=0;		
	dtheta=0;
	ctheta=0;
	Lost_SYN_Num=0;
	Rec_Samples=0;
	bzero(mem,FL2*sizeof(fcomplex));
}  

fcomplex SynDetect::convshift(fcomplex input)
{
	mem[0]=input;

	//如果 mFL是奇数，那么会多算一次 0*0 的运算	
	fcomplex conv;
	muladdc_sse((float *)mem,(float *)mTaps,(float *)&conv,FL2,add_sse);		
	for(Uint4 k=1;k<mFL;k++){
		mem[mFL-k]=mem[mFL-k-1];
	}
	return conv;
}

fcomplex SynDetect::convshift(fcomplex input, float *energy)
{
	mem[0]=input;
	*energy=*energy+(input.norm2()-mem[mFL-1].norm2())/(mFL-1);

	//如果 mFL是奇数，那么会多算一次 0*0 的运算	
	
	fcomplex conv;
	muladdc_sse((float *)mem,(float *)mTaps,(float *)&conv,FL2,add_sse);	
		
	for(Uint4 k=1;k<mFL;k++){
		mem[mFL-k]=mem[mFL-k-1];
	}
	return conv;
}
/*
 * 初始频率偏差的估计和校准
 */
 
float SynDetect::estimateFreq(int offset)
{
	assert(offset<=mSPS);
	fcomplex *ssig=new fcomplex [mFL];
	int L2=mFL/mSPS-1;	
	//同步码最佳采样
	for(Uint4 i=0;i<L2;i++){
		ssig[i]=mem[i*mSPS+offset]*mTaps[i*mSPS];
		//cout<<fast_atan2f(ssig[i].imag(),ssig[i].real())<<endl;
	}
	fcomplex part1=0;
	fcomplex part2=0;
	int DL=2;
	for(Uint4 i=0;i<L2-DL;i++){
		part1=part1+ssig[i];
	}		
	for(Uint4 i=DL;i<L2;i++){
		part2=part2+ssig[i];
	}
	
	//相位差估计==频差估计
	fcomplex dpart=(part2*part1.conj());
	 float etheta=fast_atan2f(dpart.imag(),dpart.real())/DL/mSPS; //需要优化
	 
	fcomplex ref=0;
	for(Uint4 i=0;i<mFL;i++){
		ref=ref+mTaps[i].norm2()*expj(i*etheta); //需要优化
	}
	
	assert(TapEN>0);
	chan=chan*ref/TapEN;
	//chan=chan.abs()*expj(-fast_atan2f(ssig[L2].imag(),ssig[L2].real()));
	delete (fcomplex *) ssig;
	float deltaf=-etheta/2/M_PI;
	return deltaf;

}

void SynDetect::estimateEnergy(fcomplex input, float *energy)
{
	mem[0]=input;
	*energy=*energy+(input.norm2()-mem[mFL-1].norm2())/(mFL-1); 
	
	for(Uint4 k=1;k<mFL;k++){
		mem[mFL-k]=mem[mFL-k-1];
	}
}      

  

DigitalDeMod::DigitalDeMod(DigPram Pram)
:mMod(Pram.Mod),mPulse(Pram.Pulse),mSPS(Pram.SPS),mTS(Pram.TS)
{
	assert(Pram.SynBits>0);
	assert(Pram.DataBits>0);
	
	initMapper(); //get mapper and mBPS;
	//create demode
	dMod=new DigitalMod(Pram);	

	//create filter GR
	firGR= new filter_cf(mTS);

	//create Syn detector
	if(((Pram.SynBits % mBPS)!=0) or((Pram.DataBits % mBPS)!=0))
	cout<<"Warning! SYNDetetect Create: input bits are incompatiable with mBPS "<<endl;
	
	assert(mBPS>0);	
	SynDT=new SynDetect(Pram.SynBits*mSPS/mBPS,Pram.DataBits*mSPS/mBPS,mSPS);
	
	//init GR taps
	std::vector<float> taps;	
	switch(mPulse){
		case RT_RCOSINE:taps=firdes.root_raised_cosine(1,mSPS,1,Pram.alpha,Pram.TS);break;
		case SQUARE:taps.assign(mTS,1.0/mTS);break;
		default:;break;
	}
	plltap=firdes.low_pass(1,1000,200,300,fir_design::WIN_HAMMING,0);
	cout<<"plltap= "<<plltap.size()<<" ";
	for(Uint4 i=0;i<plltap.size();i++) cout<<plltap[i]<<" ";
	cout<<endl;
	float power=0;
	mES=0;
	for(Uint4 i=0;i<taps.size();i++){
		power=power+taps[i]*taps[i];
	}
	mES=power;
	//assume most power in one SPS
	pulsepower=power*power;
	//cout<<pulsepower<<endl;
	
	firGR->setTaps(taps);
	
	reset();//复位所有数据
	
	

	if(Pram.Mod==FSK){
		assert(Pram.FSKRate>0);
		setDeltaW(Pram.FSKRate);
	}
	counter=0;
}

DigitalDeMod::~DigitalDeMod()
{
	delete (DigitalMod *) dMod;
	delete (filter_cf *) firGR;
	delete (SynDetect *) SynDT;
}
/*
110 starData=6 (-1 1.22461e-16j)
000 starData=0 (1 0j)
101 starData=5 (-1.83691e-16 -1j)
111 starData=7 (-0.707107 -0.707107j)*/

std::vector <BYTE> DigitalDeMod::work(fcomplex *input,int Len)
{
	fcomplex *sigAfGR= new fcomplex[Len]; //singal after GR filter
	fcomplex *output=new fcomplex[Len];
	
	firGR->filter(input,sigAfGR,Len);
	std::vector<SynInfo> syninfo=
				SynDT->work(sigAfGR,output,Len);
	//ploter.plot(output,Len);
	//while(1){}				

	std::vector<fcomplex> stars=DeModStar(syninfo,sigAfGR);
	
	//ploter.plotstar(stars);
	//while(1){}

				
	std::vector<BYTE> bits=DeModulate(syninfo,sigAfGR,&stars);	

	cout<<"first="<<syninfo[0].index<<" "<<syninfo[0].chan<<endl;	
	//ploter.plot(sigAfGR+syninfo[0].index,64);		
			
	for(Uint4 i=0;i<bits.size();i++) cout<<(int)bits[i];
	cout<<endl;
	delete (fcomplex *)sigAfGR;
	delete (fcomplex *)output;
	
	return bits;
	
}

std::vector <BYTE> DigitalDeMod::work(complex16 *input,int Len, bool *freqok,FcVector *pstars,float *toa)
{
	fcomplex *finput = new fcomplex[Len]; 
	fcomplex *sigAfGR= new fcomplex[Len]; //singal after GR filter
	fcomplex *output=new fcomplex[Len];
	
	for(Uint4 i=0;i<Len;i++) finput[i]=fcomplex(input[i].real(),input[i].imag());
	  

	
	//cout<<"filter"<<endl;
	firGR->filter(finput,sigAfGR,Len);
	//cout<<"syn"<<endl;
	std::vector<SynInfo> syninfo=SynDT->work(sigAfGR,output,Len);
	*toa=SynDT->getToa();
	
	if(SynDT->LSN()>=500){ 
		*freqok=false;
		SynDT->clear();
		cout<<"同步头丢失，重新复位接收机"<<endl;
	}
	/*
	counter++;
	if(counter==100)
	{
		float real[Len];
		for(Uint4 i=0;i<Len;i++) {real[i]=sigAfGR[i].abs();};
		ploter.plot(real,Len);
		counter=(random()%10);
		
		
	}*/
		
	std::vector<BYTE> bits=DeModulate(syninfo,sigAfGR,pstars);	


	delete (fcomplex *)sigAfGR;
	delete (fcomplex *)output;
	delete (fcomplex *)finput;
	return bits;
	
}
void DigitalDeMod::initSynDetect(BYTE *SynBits,Uint4 Synlen)
{
	dMod->reset();
	std::vector<fcomplex> SynSig=dMod->ModulateGTGR(SynBits,Synlen);
	
	float ESyn=0;
	float Avpower=0;

	for(Uint4 i=0;i<SynSig.size();i++){
	//	cout<<i<<" "<<SynSig[i]<<endl;
		ESyn=ESyn+SynSig[i].norm2();
	}

	Avpower=ESyn/SynSig.size();
	cout<<"ESyn="<<ESyn<<" Avpower="<<Avpower<<endl;
	float CTH=ESyn*ESyn/Avpower;
	cout<<"CTH="<<CTH<<endl;
	SynDT->setTaps(SynSig);
	SynDT->setPTH(0);
	SynDT->setADJ(sqrt(Avpower)/1.5);// i don't konw why 
	
	
	switch(mMod){
		case ASK: SynDT->setCTH(CTH*0.7);break; // 由于ASK 影响相关峰值的尖锐程度，所以必须要提高一些电平
		case QPSK:;
		case PSK8:;
		case BPSK:SynDT->setCTH(CTH*0.5);break;
		case FSK:SynDT->setCTH(CTH*0.6);break;
	}
	
}

void DigitalDeMod::initMapper()
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
			lastsig=1;
		default:mBPS=1;break;
	}  
}  
std::vector <BYTE> DigitalDeMod::DeModulate(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars)
{
	std::vector <BYTE> bits;
	switch(mMod){
		case ASK:bits=DeModulateLinear(syninfo,signal,stars);break;
		case BPSK:bits=DeModulateLinear(syninfo,signal,stars);break;
		case QPSK:bits=DeModulateLinear(syninfo,signal,stars);break;
		case PSK8:bits=DeModulateLinear(syninfo,signal,stars);break;
		case FSK:bits=DeModulateFSK(syninfo,signal,stars);break;
		default:break;
		break;
	}
	return bits;
}

std::vector <BYTE> DigitalDeMod::DeModulateLinear(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars)
{
	Uint4 MapSize=(1<<mBPS);
	fcomplex sig;
	float mind,distance;  
	BYTE Decision;
	std::vector<BYTE> bits;
	for(Uint4 i=0;i<syninfo.size();i++){
		if (syninfo[i].begin) { 
			//cout<<"end="<<deltaf*200e3<<endl;	
			
			deltaf=syninfo[i].deltaf;
			//cout<<"begin="<<deltaf*200e3<<endl;
			//deltaf=0;
			dtheta=0;
			ctheta=0;

		}
		if (mPhaseTrack)
			ctheta=2*M_PI*deltaf*mSPS+ctheta;
			
			
		sig=signal[syninfo[i].index]*syninfo[i].chan*expj(-ctheta);
		(*stars).push_back(sig);
		mind=(sig-Mapper[0]).norm2();	
		Decision=0;
		for(Uint4 j=1;j<MapSize;j++){
			distance=(sig-Mapper[j]).norm2();
			if(distance<mind){
				mind=distance;
				Decision=j;
			}		  
		}

		fcomplex temp=sig*Mapper[Decision].conj();
		dtheta=fast_atan2f(temp.imag(),temp.real());
		
		if (mPhaseTrack)
			deltaf=deltaf+dtheta/5000;

		
		
		//cout<<"Decesion="<<(int)Decision<<" sig="<<sig<<endl;
		BYTE outbits;
		for(Uint4 j=1;j<=mBPS;j++){
			outbits=(Decision>>(mBPS-j)) & 0x1;
			bits.push_back(outbits);
		}
		//cout<<(int)Decision;
		
	}
	//
	//cout<<endl;
	return bits;
	
}
std::vector <BYTE> DigitalDeMod::DeModulateFSK(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars)
{
	Uint4 MapSize=(1<<mBPS);
	fcomplex sig;
	float mind,distance;
	BYTE Decision;
	std::vector<BYTE> bits;
	for(Uint4 i=0;i<syninfo.size();i++){
		sig=signal[syninfo[i].index]*syninfo[i].chan;
		lastsig=signal[syninfo[i].index-mSPS+1]*syninfo[i].chan;
		fcomplex freqsig=sig*lastsig.conj();		
		if(freqsig.real()/freqsig.imag()>0) Decision=0;else Decision=1;
		//cout<<freqsig<<endl;
		//cout<<"Decesion="<<(int)Decision<<" sig="<<sig<<endl;
		BYTE outbits;
		for(Uint4 j=1;j<=mBPS;j++){
			outbits=(Decision>>(mBPS-j)) & 0x1;
			bits.push_back(outbits);
		}
		//cout<<(int)Decision;
		
	}
	//cout<<endl;
	return bits;
	
}
std::vector <fcomplex> DigitalDeMod::DeModStar(std::vector<SynInfo> syninfo, fcomplex *signal)
{
	std::vector<fcomplex> stars;
	fcomplex sig;
	for(Uint4 i=0;i<syninfo.size();i++){
		sig=signal[syninfo[i].index]*syninfo[i].chan;
		stars.push_back(sig);
		//cout<<i<<" "<<sig;
	}
	return stars;
}

void DigitalDeMod::reset()
{
	SynDT->clear();
	dMod->reset();
	firGR->clear();
}


