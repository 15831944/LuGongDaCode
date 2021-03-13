
#include <iostream>
#include <string.h>  
#include <math.h>
#include <assert.h>


#include <dsp/SSE.h>
#include <dsp/Filter.h>

using namespace std;



  

filter_cc::filter_cc(int FL)
{
	mFL=FL;
	if((mFL%2)!=0) FL2=mFL+1;else FL2=mFL;
	mTaps= (fcomplex *) malloc_with_16bytes_alignment(FL2*sizeof(fcomplex));
	mem =(fcomplex *)  malloc_with_16bytes_alignment(FL2*sizeof(fcomplex)*2);
	add_sse=(float *) malloc_with_16bytes_alignment(4*sizeof(float));
	
	

	//cout<<"taps="<<(int)taps % 16<<endl;
	//cout<<"mem="<<(int )mem % 16 <<endl;
	//cout<<"add_sse="<<(int )add_sse % 16 <<endl;
	bzero(mTaps,FL2*sizeof(fcomplex));
	bzero(mem,FL2*sizeof(fcomplex));
	pos=FL;
	

}

void filter_cc::clear()
{
	bzero(mem,mFL*sizeof(fcomplex));
	pos=mFL;
}


filter_cc::~filter_cc()
{
	free_with_16bytes_alignment(mTaps);
	free_with_16bytes_alignment(mem);
	free_with_16bytes_alignment(add_sse);
}



void filter_cc::setTaps(fcomplex *taps)
{
	memcpy(mTaps,taps,mFL*sizeof(fcomplex));	
}
void filter_cc::setTaps(std::vector<fcomplex> taps)
{
	assert(taps.size()==mFL);
	for(Uint4 i=0;i<taps.size();i++) mTaps[i]=taps[i];
}

void filter_cc::filter_omp(fcomplex *input, fcomplex *output, int Len)
{
	int i;	
	#pragma omp parallel for 	 	
	for(i=0;i<Len;i++){
		mem[pos]=input[i];
		fcomplex *tmem;
		tmem=mem+pos;	
		output[i]=0;		
		//如果 mFL是奇数，那么会多算一次 tmem*0 的运算		
		ccomplex_dotprod_generic((float *)tmem,(float *)mTaps,FL2/2,(float *)(output+i));		
		pos--;
		if (pos<0){
			memcpy(mem+mFL+1,mem,(mFL-1)*sizeof(fcomplex));
			pos=mFL;
		}
	}		
	  
}

void filter_cc::filter_sse(fcomplex *input, fcomplex *output, int Len)
{
	int i,k;  
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		//如果 mFL是奇数，那么会多算一次 0*0 的运算	
		muladdc_sse((float *)mem,(float *)mTaps,(float *)(output+i),FL2,add_sse);		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}		
}




void filter_cc::filter(fcomplex *input, fcomplex *output, int Len)
{
	int i,k;	
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		output[i]=0;
		for(k=0;k<mFL;k++){
			output[i]+=mTaps[k]*mem[k];
		} 		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}	
}








///////////////////////////////////////////////////////////////////
////===============filter_ff==================================////
 

filter_ff::filter_ff(int FL)
{
	mFL=FL;
	if((mFL%2)!=0) FL2=mFL+1;else FL2=mFL;
	mTaps= (float *) malloc_with_16bytes_alignment(FL2*sizeof(float));
	mem =(float *)  malloc_with_16bytes_alignment(FL2*sizeof(float)*2);
	add_sse=(float *) malloc_with_16bytes_alignment(4*sizeof(float));
	

	bzero(mTaps,FL2*sizeof(float));
	bzero(mem,FL2*sizeof(float));
	pos=FL;
	

}

void filter_ff::clear()
{
	bzero(mem,mFL*sizeof(float));
	pos=mFL;
}


filter_ff::~filter_ff()
{
	free_with_16bytes_alignment(mTaps);
	free_with_16bytes_alignment(mem);
	free_with_16bytes_alignment(add_sse);
}



void filter_ff::setTaps(float *taps)
{
	memcpy(mTaps,taps,mFL*sizeof(float));	
}
void filter_ff::setTaps(std::vector<float> taps){
	for(Uint4 i=0;i<taps.size();i++) mTaps[i]=taps[i];
}
void filter_ff::filter_omp(float *input, float *output, int Len)
{
/*	int i;	
	#pragma omp parallel for 	 	
	for(i=0;i<Len;i++){
		mem[pos]=input[i];
		fcomplex *tmem;
		tmem=mem+pos;	
		output[i]=0;		
		//如果 mFL是奇数，那么会多算一次 tmem*0 的运算		
		ccomplex_dotprod_generic((float *)tmem,(float *)mTaps,FL2/2,(float *)(output+i));		
		pos--;
		if (pos<0){
			memcpy(mem+mFL+1,mem,(mFL-1)*sizeof(fcomplex));
			pos=mFL;
		}
	}		
*/	  
}

void filter_ff::filter_sse(float *input, float *output, int Len)
{
	/*
	int i,k;  
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		//如果 mFL是奇数，那么会多算一次 0*0 的运算	
		muladdc_sse((float *)mem,(float *)mTaps,(float *)(output+i),FL2,add_sse);		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}*/		
}




void filter_ff::filter(float *input, float *output, int Len)
{
	int i,k;	
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		output[i]=0;
		for(k=0;k<mFL;k++){
			output[i]+=mTaps[k]*mem[k];
		} 		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}	
}



////////////////////////////////////////////////////////////////////////
/////====================filter_cf====================================//



filter_cf::filter_cf(int FL)
{
	mFL=FL;
	if((mFL%2)!=0) FL2=mFL+1;else FL2=mFL;
	mTaps= (float *) malloc_with_16bytes_alignment(FL2*sizeof(float));
	mem =(fcomplex *)  malloc_with_16bytes_alignment(FL2*sizeof(fcomplex)*2);
	add_sse=(float *) malloc_with_16bytes_alignment(4*sizeof(float));
	
	

	//cout<<"taps="<<(int)taps % 16<<endl;
	//cout<<"mem="<<(int )mem % 16 <<endl;
	//cout<<"add_sse="<<(int )add_sse % 16 <<endl;
	bzero(mTaps,FL2*sizeof(float));
	bzero(mem,FL2*sizeof(fcomplex));
	pos=FL;
	

}

void filter_cf::clear()
{
	bzero(mem,mFL*sizeof(fcomplex));
	pos=mFL;
}


filter_cf::~filter_cf()
{
	free_with_16bytes_alignment(mTaps);
	free_with_16bytes_alignment(mem);
	free_with_16bytes_alignment(add_sse);
}



void filter_cf::setTaps(float *taps)
{
	memcpy(mTaps,taps,mFL*sizeof(float));	
}

void filter_cf::setTaps(std::vector<float> taps){
	for(Uint4 i=0;i<taps.size();i++) mTaps[i]=taps[i];
}

void filter_cf::filter_omp(fcomplex *input, fcomplex *output, int Len)
{
	/*
	int i;	
	#pragma omp parallel for 	 	
	for(i=0;i<Len;i++){
		mem[pos]=input[i];
		fcomplex *tmem;
		tmem=mem+pos;	
		output[i]=0;		
		//如果 mFL是奇数，那么会多算一次 tmem*0 的运算		
		ccomplex_dotprod_generic((float *)tmem,(float *)mTaps,FL2/2,(float *)(output+i));		
		pos--;
		if (pos<0){
			memcpy(mem+mFL+1,mem,(mFL-1)*sizeof(fcomplex));
			pos=mFL;
		}
	}		
	 */ 
}

void filter_cf::filter_sse(fcomplex *input, fcomplex *output, int Len)
{
/*	
	int i,k;  
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		//如果 mFL是奇数，那么会多算一次 0*0 的运算	
		muladdc_sse((float *)mem,(float *)mTaps,(float *)(output+i),FL2,add_sse);		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}	
	*/	
}




void filter_cf::filter(fcomplex *input, fcomplex *output, int Len)
{
	int i,k;	
	for(i=0;i<Len;i++){
		mem[0]=input[i];
		output[i]=0;
		for(k=0;k<mFL;k++){
			output[i]+=mTaps[k]*mem[k];
		} 		
		for(k=1;k<mFL;k++){
			mem[mFL-k]=mem[mFL-k-1];
		}
	}	
}

