#ifndef SSE_H
#define SSE_H

#include <xmmintrin.h>
#include <stdlib.h>
#include <common/Types.hpp>



/*
 * example
 * x[i]=a[2*i]+a[2*i+1]*j;
 * y[i]=b[2*i]+b[2*i+1]*j;
 * z[i]=x[i]*y[i]
 * c[2*i]=z[i].real();c[2*i+1]=z[i].imag(),
 * i=0,1,2...Len/2
 */ 
void mulc_sse(float *a, float *b, float *c, int Len);



/*
 * example
 * x[i]=a[2*i]+a[2*i+1]*j;
 * y[i]=b[2*i]+b[2*i+1]*j;
 * z=z+x[i]*y[i]
 * c[0]=z.real();c[1]=z.imag(),
 * i=0,1,2...Len/2
 */
void muladdc_sse(float *a, float *b, float *c, int Len, float *tc);


void showm128(__m128 x,const char *name);
void *malloc_with_16bytes_alignment(size_t size);
void free_with_16bytes_alignment(void *memblock);


void ccomplex_dotprod_generic (const float *input,const float *taps, unsigned n_2_ccomplex_blocks, float *result);

Uint4 graycode(Uint4 data, Uint4 width);
#endif
