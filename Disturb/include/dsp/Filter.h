#ifndef FILTER_H
#define FILTER_H

#include <stdlib.h>
#include <vector>

#include <common/Types.hpp>
#include <common/Complex.hpp>
#include <dsp/SSE.h>


//输入复数，系数复数
class filter_cc
{
protected:
	fcomplex 	*mTaps;  
	fcomplex 	*mem;
	float      *add_sse;
	int		  	mFL;
	int			FL2;
	int			pos;
public:
	filter_cc(int FL);
	~filter_cc();
void setTaps(fcomplex *taps);
void setTaps(std::vector<fcomplex> taps);
void filter_sse(fcomplex *input,fcomplex *output, int Len);
void filter_omp(fcomplex *input,fcomplex *output, int Len);
void filter(fcomplex *input, fcomplex *output, int Len);
void clear();


};

//输入float，系数float
class filter_ff
{
protected:
	float 		*mTaps;  
	float 		*mem;
	float      *add_sse;
	int		  	mFL;
	int			FL2;
	int			pos;
public:
	filter_ff(int FL);
	~filter_ff();
void setTaps(float *taps);
void setTaps(std::vector<float> taps);
void filter_sse(float *input,float *output, int Len);
void filter_omp(float *input,float *output, int Len);
void filter(float *input, float *output, int Len);
void clear();


};

//输入complex，系数float
class filter_cf
{
protected:
	float 		*mTaps;  
	fcomplex 	*mem;
	float      *add_sse;
	int		  	mFL;
	int			FL2;
	int			pos;
public:
	filter_cf(int FL);
	~filter_cf();
void setTaps(float *taps);
void setTaps(std::vector<float> taps);
void filter_sse(fcomplex *input,fcomplex *output, int Len);
void filter_omp(fcomplex *input,fcomplex *output, int Len);
void filter(fcomplex *input, fcomplex *output, int Len);
void clear();
};


#endif

