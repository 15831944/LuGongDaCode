#ifndef DIGITAL_DEMOD_H
#define DIGITAL_DEMOD_H

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>
#include <string.h>
#include <vector>
#include <assert.h>
#include <common/Types.hpp>
#include <common/Complex.hpp>
#include <common/GNUPlot.h>
#include <dsp/SSE.h>
#include <dsp/FirDesign.h>
#include <dsp/Filter.h>
#include <dsp/DigitalSig.h>
#include <dsp/DigitalMod.h>

typedef struct{
	Uint4 index;
	fcomplex chan;
	bool begin;
	float deltaf;
}SynInfo;

class SynDetect
{
enum SYNST {EN_DETECT,SYN_NO,SYN_MAY,SYN_OK};
protected:
	Uint4		mFL; //扩频码的总样值数
	Uint4		mDL; //数据的样值数
	Uint4 		mSPS;// samples per symbol
	Uint4 		mT; //mFL+mDL
	fcomplex 	*mTaps;  //同步码抽头系数
	fcomplex 	*mem;  //移位寄存器
	fcomplex    *convbuf;
	unsigned char convbufpos;//256 的大小的移位寄存器，不用 % 操作
	
	float      *add_sse; //sse 运算时所需的中间变量，于提升运算效率有关
	int			FL2; // ceil(mFL/2)*2; 
	SYNST		state; //同步器的状态

	Uint4	    needwatch; //由于相关值大于门限后，并不表示是最佳采样处，所以需要观察其后续几个样值
	int			needwait; //确定最大峰值后的时刻所对应样值之后几个样值就是数据
	float		maxpeak; //最大峰值
	float		avpower; //接收信号的平均功率
	Uint4		counter; //接收样值计数器,用于确定何时重启同步
	
	int 		pos;    //记录最大峰值对应的位置
	int 		posPeakbuf;//记录峰值在相关值移位寄存器中的位置
	fcomplex    chan;  //信道估计
	float		mPTH; //power threshold
	float      mCTH; //conv threshold
	float      mADJ;// adjust with pulse sytle
	
	float 		dtheta; //频率差所带来的每个样值的相位差
	float      ctheta; //累计相位误差

	
	float      TapEN; //抽头系数能量
	Uint4 	 	Lost_SYN_Num;
	Uint4		Rec_Samples;
	
	bool 		begin;
	float      deltaf;
	
	
	bool 		toaestimate;
	float 		toa;
	
public:
	SynDetect(Uint4 FL, Uint4 DL, Uint4 SPS);
	~SynDetect();
	void setTaps(fcomplex *taps);
	void setTaps(std::vector<fcomplex> taps);
	std::vector<SynInfo> work_test(fcomplex *input,float *output, int Len);
	std::vector<SynInfo> work(fcomplex *input,fcomplex *output, int Len);
	void clear();
	void setPTH(float PTH){mPTH=PTH;};
	void setCTH(float CTH){mCTH=CTH;};
	void setADJ(float ADJ){mADJ=ADJ;};
	float getToa(){return toa;}
	
	Uint4 LSN(){return Lost_SYN_Num;}
	
	fcomplex convshift(fcomplex input);
	fcomplex convshift(fcomplex input, float *energy);
	void  estimateEnergy(fcomplex input, float *energy);
	float estimateFreq(int offset);
	Uint4 T(){return mT;}



};



class DigitalDeMod
{
//enum DEMODST {EN_DETECT,FREQ_ADJUST,NORMAL_WORK};
protected:
	DigMOD mMod;
	PULSE mPulse;
	Uint4 mSPS; //samples per symbol
	Uint4 mTS; //duration of one symbol
	Uint4 mBPS;//bits per symbol
	
	Uint4 mRadix;
	DigitalMod *dMod;//Modlation part for SYN or Decision Feedback LPC
	filter_cf *firGR; //Match filter of Receiver
	SynDetect  *SynDT;
	fir_design firdes;
	fcomplex Mapper[128]; //the bigest is 128QAM
	Uint4	 gcode[128];	
	float	 pulsepower;
	float	 mES;
	float 	 mDeltaW;
	
	float   dtheta;
	fcomplex lastsig;
	float   deltaf;
	float 	 ctheta;
	float   cdtheta;
	std::vector<float> plltap;   
	void  (*kalfreq)(float deltaf);
	bool 	 mPhaseTrack;
	int counter;
public:
	DigitalDeMod(DigPram Pram);
	~DigitalDeMod();
	Uint4 BPS(){return mBPS;}
	void initMapper();
	void reset();
	void setPhaseTrack(bool wPhaseTrack){mPhaseTrack=wPhaseTrack;}
	void initSynDetect(BYTE *SynBits,Uint4 Synlen);
	std::vector <BYTE> work(fcomplex *input,int Len);
	std::vector <BYTE> work(complex16 *input,int Len, bool *freqok, FcVector *pstars,float *toa);
	void setDeltaW(float DeltaW){mDeltaW=DeltaW;dMod->setDeltaW(mDeltaW);}
	std::vector <BYTE> DeModulate(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars);
	std::vector <fcomplex> DeModStar(std::vector<SynInfo> syninfo, fcomplex *signal);
	std::vector <BYTE> DeModulateLinear(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars);//线性调制
	std::vector <BYTE> DeModulateFSK(std::vector<SynInfo> syninfo, fcomplex *signal,std::vector<fcomplex> *stars);//线性调制
};






#endif
