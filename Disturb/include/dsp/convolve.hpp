#include <malloc.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
using namespace std;
void mac_real(float *x, float *h, float *y)
{
	y[0] += x[0] * h[0];
	//y[1] += x[1] * h[0];	
}


void mac_cmplx(float *x, float *h, float *y)
{
	y[0] += x[0] * h[0] - x[1] * h[1];
	y[1] += x[0] * h[1] + x[1] * h[0]; 
}

static void mac_real_vec_n(float *x, float *h, float *y,
						   int len, int step, int offset)
{
	for (int i = offset; i < len; i += step)
	{	
		mac_real(&x[i], &h[i], y);
	}

}

/* Base vector complex-complex multiply and accumulate */
static void mac_cmplx_vec_n(float *x, float *h, float *y,
							int h_len, int step, int offset)
{
	for (int i = offset; i <h_len; i += step)
	{
		mac_cmplx(&x[2 * i], &h[2 * i], y);
		//cout<<"x["<<2*i<<"]="<<x[2*i]<<"  h["<<2*i<<"]="<<h[2*i]<<"  y["<<i<<"]="<<y[i]<<endl;
	} 
}

/* Base complex-real convolution */
static int _base_convolve_real(float *x, int x_len,
							   float *h, int h_len,
							   float *y, int y_len,
							   int start, int len,
							   int step, int offset)
{
	for (int i = 0; i < len; i++) 
	{
		mac_real_vec_n(&x[(i - (h_len - 1) + start)],h,&y[i], h_len,step, offset);
	}
	return len;
}

/* Base complex-complex convolution */
static int _base_convolve_complex(float *x, int x_len,
								  float *h, int h_len,
								  float *y, int y_len,
								  int start, int len,
								  int step, int offset)
{
	for (int i = 0; i < len; i++) 
	{
		mac_cmplx_vec_n(&x[2 * (i - (h_len - 1) + start)],h,&y[2 * i],h_len, step, offset);
	}
	return len;
}


static int bounds_check(int x_len, int h_len, int y_len,
						int start, int len, int step)
{
	if ((x_len < 1) || (h_len < 1) ||(y_len < 1) || (len < 1) || (step < 1)) 
	{
		fprintf(stderr, "Convolve: Invalid input\n");
		return -1;
	}

	if ((start + len > x_len) || (len > y_len) || (x_len < h_len)) 
	{
		fprintf(stderr, "Convolve: Boundary exception\n");
		fprintf(stderr, "start: %i, len: %i, x: %i, h: %i, y: %i\n",start, len, x_len, h_len, y_len);
		return -1;
	}

	return 0;
}

int convolve_real(float *x, int x_len,
				  float *h, int h_len,
				  float *y, int y_len,
				  int start, int len,
				  int step, int offset)
{
	void (*conv_func)(float *, float *, float *, int) = NULL;
	void (*conv_func_n)(float *, float *, float *, int, int) = NULL;

	if (bounds_check(x_len, h_len, y_len, start, len, step) < 0)
		return -1;
	
	//void *memset(void *s,int c,size_t n);将已开辟内存空间 s 的首 n 个字节的值设为值 c。
	memset(y, 0, len * 2 * sizeof(float));
	
	if (conv_func) 
	{
		conv_func(&x[2 * (-(h_len - 1) + start)],h, y, len);
	} 
	else if (conv_func_n) 
		{
			conv_func_n(&x[2 * (-(h_len - 1) + start)],h, y, h_len, len);
		} 
		else 
		{
			_base_convolve_real(x, x_len,h, h_len,y, y_len,start, len, step, offset);
		}

	return len;
}


int convolve_complex(float *x, int x_len,
					 float *h, int h_len,
					 float *y, int y_len,
					 int start, int len,
					 int step, int offset)
{
	void (*conv_func)(float *, float *, float *, int, int) = NULL;

	if (bounds_check(x_len, h_len, y_len, start, len, step) < 0)
		return -1;

	memset(y, 0, len * 2 * sizeof(float));

	if (conv_func) 
	{
		conv_func(&x[2 * (-(h_len - 1) + start)],h, y, h_len, len);
	} 
	else 
	{
		_base_convolve_complex(x, x_len,
								h, h_len,
								y, y_len,
								start, len, step, offset);
	}
	return len;
}

int base_convolve_real(float *x, int x_len,
					   float *h, int h_len,
					   float *y, int y_len,
					   int start, int len,
					   int step, int offset)
{
	if (bounds_check(x_len, h_len, y_len, start, len, step) < 0)
		return -1;

	memset(y, 0, len * 2 * sizeof(float));

	return _base_convolve_real(x, x_len,
							   h, h_len,
							   y, y_len,
							   start, len, step, offset);
}

/* API: Non-aligned (no SSE) complex-complex */
int base_convolve_complex(float *x, int x_len,
						  float *h, int h_len,
						  float *y, int y_len,
						  int start, int len,
						  int step, int offset)
{
	if (bounds_check(x_len, h_len, y_len, start, len, step) < 0)
		return -1;

	memset(y, 0, len * 2 * sizeof(float));

	return _base_convolve_complex(x, x_len,
							  h, h_len,
							  y, y_len,
							  start, len, step, offset);
}	

