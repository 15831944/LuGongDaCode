#ifndef DFT_H
#define DFT_H
#include <fftw3.h>
#include <common/Complex.hpp>

class dfft
{
public:
	enum fftmstate {MAKE,FREE};
protected:
	dcomplex*           in;  //fft input
	dcomplex*           out; //fft ouput
	fftw_plan          fftplan;   //fft way
	unsigned int       fftsize;//fft size
	fftmstate          mstate;  //a sign of the state of dfft module.
	double 			*hwin;
public:
	dfft();
	~dfft();
	void make(unsigned mfftsize,int ws);//construct fft
	void free();// release in, out and fftplan 
	void input(short *buf);
	void input(int8_t *buf);
	void win();
	void input(fcomplex *buf);
	
	void excute();
	fftmstate readfftmstate(){return mstate;}
	dcomplex *output(){return out;};
};

#endif
