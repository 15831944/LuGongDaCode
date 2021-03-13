#ifndef DIGITAL_SIG_H
#define DIGITAL_SIG_H

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sys/time.h>
#include <string.h>
#include <vector>
#include <assert.h>
#include <common/Types.hpp>
#include <dsp/SSE.h>


enum DigMOD {ASK=0,BPSK,FSK,QPSK,PSK8};

enum PULSE {RT_RCOSINE=0,RCOSINE,SQUARE};

enum PRBS {PRBS4=0,PRBS5,PRBS6,PRBS9,PRBS11,FILE1,FILE2,FILE3};


typedef struct{
	DigMOD Mod; 
	
	PULSE Pulse;
	Uint4 TS;
	float alpha;
	
	Uint4 SPS;
	float Rb;  //信息速率
	float Rs;  //符号速率
	float Fs;  //采样率
	
	float FSKRate;
	
	Uint4 SynBits;
	Uint4 DataBits;
	
	PRBS SynFormat;
	PRBS DataFormat;
	
		
}DigPram;

void showDigPram(DigPram dPram);

Uint4 ModtoBPS(DigMOD Mod);

#endif
