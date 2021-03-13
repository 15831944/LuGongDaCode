/*
 * Rational Sample Rate Conversion
 * Copyright (C) 2012, 2013  Thomas Tsou <tom@tsou.cc>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <malloc.h>
#include <iostream>
using namespace std;
#include "dsp/Resampler.hpp"

#include "dsp/convolve.hpp"

#ifndef M_PI
#define M_PI	3.14159265358979323846264338327f
#endif

#define MAX_OUTPUT_LEN		4096

static float sinc(float x)//内插函数，sinc信号在频域上是一个矩形窗
{
	if (x == 0.0)
		return 0.9999999999;

	return sin(M_PI * x) / (M_PI * x);
}

bool Resampler::initFilters(float bw)///
{
	size_t proto_len = p * filt_len;//proto_len为滤波器插值后的长度
	float *proto, val, cutoff;
	float sum = 0.0f, scale = 0.0f;
	float midpt = (float) (proto_len - 1.0) / 2.0;

	/* 
	  Allocate partition filters and the temporary prototype filter
	  according to numerator of the rational rate. Coefficients are
	  real only and must be 16-byte memory aligned for SSE usage.
	 */
	proto = new float[proto_len];
	if (!proto)
		return false;

	partitions = (float **) malloc(sizeof(float *) * p);
	if (!partitions) 
	{
		free(proto);
		return false;
	}

	for (size_t i = 0; i < p; i++) 
	{
		partitions[i] = (float *)
		
		//memalign (16, size_t size)分配一个大小为size，地址是16的倍数的内存块
		memalign(16, filt_len * 2 * sizeof(float));
		//memalign(16, filt_len  * sizeof(float));
	}

	/* 
	  Generate the prototype filter with a Blackman-harris window(用布莱克曼窗生成一个滤波器模型).
	  Scale coefficients with DC filter gain set to unity divided
	  by the number of filter partitions. 
	 */
	float a0 = 0.35875;
	float a1 = 0.48829;
	float a2 = 0.14128;
	float a3 = 0.01168;

	if (p > q)
		cutoff = (float) p;
	else
		cutoff = (float) q;

	for (size_t i = 0; i < proto_len; i++) 
	{
		proto[i] = sinc(((float) i - midpt) / cutoff * bw);//采用sinc信号进行插值,从而重建出原始信号的一个近似，插值点在midpt处
		proto[i] *= a0 -
					a1 * cos(2 * M_PI * i / (proto_len - 1)) +
					a2 * cos(4 * M_PI * i / (proto_len - 1)) -
					a3 * cos(6 * M_PI * i / (proto_len - 1));
		sum += proto[i];
	}
	scale = p / sum;//比例系数

	/* Populate filter partitions from the prototype filter */
	for (size_t i = 0; i < filt_len; i++) //生成标准滤波表
	{
		for (size_t n = 0; n < p; n++) 
		{
			partitions[n][2 * i + 0] = proto[i * p + n] * scale;
			partitions[n][2 * i + 1] = 0.0f;
			//partitions[n][i] = proto[i * p + n] * scale;
		}
	}

	/* For convolution, we store the filter taps in reverse */ 
	for (size_t n = 0; n < p; n++) //对于卷积以相反方式保存滤波表
	{
		for (size_t i = 0; i < filt_len / 2; i++) 
		{ 
			val = partitions[n][2 * i];
			partitions[n][2 * i] = partitions[n][2 * (filt_len - 1 - i)];
			partitions[n][2 * (filt_len - 1 - i)] = val;
			//val = partitions[n][i];
			//partitions[n][i] = partitions[n][(filt_len - 1 - i)];
			//partitions[n][(filt_len - 1 - i)] = val;
		}
	}

	delete proto;

	return true;
}

void Resampler::releaseFilters()
{
	if (partitions) 
	{
		for (size_t i = 0; i < p; i++)
			free(partitions[i]);
	}

	free(partitions);
	partitions = NULL;
}

///
static bool check_vec_len(int in_len, int out_len, int p, int q)
{
	if (in_len % q) 
	{
		std::cerr << "Invalid input length " << in_len
				  <<  " is not multiple of " << q << std::endl;
		return false;
	}

	if (out_len % p) 
	{
		std::cerr << "Invalid output length " << out_len
			  <<  " is not multiple of " << p << std::endl;
		return false;
	}

	if ((in_len / q) != (out_len / p)) 
	{
		std::cerr << "Input/output block length mismatch（不匹配）" << std::endl;
		std::cerr << "P = " << p << ", Q = " << q << std::endl;
		std::cerr << "Input len: " << in_len << std::endl;
		std::cerr << "Output len: " << out_len << std::endl;
		return false;
	}

	if (out_len > MAX_OUTPUT_LEN) 
	{
		std::cerr << "Block length of " << out_len
			  << " exceeds（超过） max of " << MAX_OUTPUT_LEN << std::endl;
		return false;
	}

	return true;
}

void Resampler::computePath()
{
	for (int i = 0; i < MAX_OUTPUT_LEN; i++) 
	{
		in_index[i] = (q * i) / p;//每个输出个数对应当前的输入个数
		out_path[i] = (q * i) % p;//对应每个块滤波表的下标
	}
}

int Resampler::rotate(float *in, size_t in_len, float *out, size_t out_len)///
{
	int n, path;
	int hist_len = filt_len - 1;

	if (!check_vec_len(in_len, out_len, p, q))
		return -1; 
    
	//memcpy(void *dest, const void *src, size_t n);
	//从源src所指的内存地址的起始位置开始拷贝n个字节到目标dest所指的内存地址的起始位置中
	memcpy(&in[-2 * hist_len], history, hist_len * 2 * sizeof(float));
	for (size_t i = 0; i < out_len; i++) 
	{
		n = in_index[i]; 
		path = out_path[i];
		base_convolve_complex(in, in_len,    //x(t),x_len
					  partitions[path], filt_len, //h(t),h_len
					  &out[2 * i], out_len - i,  //y(t),y_len
					  n, 1, 1, 0); //start,len,step,offset
	} //cout<<"path="<<out[2*i]<<endl;
	memcpy(history, &in[2 * (in_len - hist_len)],2 * hist_len * sizeof(float));
	return out_len;
}

bool Resampler::init(float bw)
{
	size_t hist_len = filt_len - 1;

	/* Filterbank filter internals */
	if (initFilters(bw) < 0)
		return false;

	/* History buffer */
	history = new float[2 * hist_len];
	memset(history, 0, 2 * hist_len * sizeof(float));//memset(void *s,int c,size_t n)将已开辟内存空间 s 的首 n 个字节的值设为值 c
    //history = new float[hist_len];
    //memset(history, 0, hist_len * sizeof(float));
    
	/* Precompute filterbank paths */
	in_index = new size_t[MAX_OUTPUT_LEN];
	out_path = new size_t[MAX_OUTPUT_LEN];
	computePath();
	return true;
}

size_t Resampler::len()
{
	return filt_len;
}

Resampler::Resampler(size_t p, size_t q, size_t filt_len)
	: in_index(NULL), out_path(NULL), partitions(NULL), history(NULL)
{
	this->p = p;
	this->q = q;
	this->filt_len = filt_len;
}

Resampler::~Resampler()
{
	releaseFilters();

	delete history;
	delete in_index;
	delete out_path;
}
