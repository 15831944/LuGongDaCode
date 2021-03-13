#include <stdlib.h>
#include <string.h>
#include <iostream>

#include <dsp/SSE.h>

using namespace std;


void ccomplex_dotprod_generic (const float *input,
                        const float *taps, unsigned n_2_ccomplex_blocks, float *result)
{
    float sum0 = 0;
    float sum1 = 0;
    float sum2 = 0;
    float sum3 = 0;
  
    do {
  
      sum0 += input[0] * taps[0] - input[1] * taps[1];
      sum1 += input[0] * taps[1] + input[1] * taps[0];
      sum2 += input[2] * taps[2] - input[3] * taps[3];
      sum3 += input[2] * taps[3] + input[3] * taps[2];
  
      input += 4;
      taps += 4;
  
    } while (--n_2_ccomplex_blocks != 0);
  
  
    result[0] = result[0]+sum0 + sum2;
    result[1] =result[1]+ sum1 + sum3;
 }

void showm128(__m128 x,const char *name)
{
	float a[4];
	_mm_store_ps(&a[0], x);
	cout<<name<<"=";
	for(int i=0;i<4;i++) cout<<a[i]<<" ";
	cout<<endl;
}     


void mulc_sse(float *a, float *b, float *c, int Len)
{
	__m128 x0, x1, x2, t0, t1, m;
	m = _mm_set_ps(-1, 1, -1, 1);
    for (int j = 0; j < Len*2; j+=4) {	// filter in the freq domain
      x0 = _mm_load_ps(&a[j]);
      t0 = _mm_load_ps(&b[j]);
      t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(3, 3, 1, 1));
      t0 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(2, 2, 0, 0));
      t1 = _mm_mul_ps(t1, m);
      x1 = _mm_mul_ps(x0, t0);
      x2 = _mm_mul_ps(x0, t1);

      x2 = _mm_shuffle_ps(x2, x2, _MM_SHUFFLE(2, 3, 0, 1));
      x2 = _mm_add_ps(x1, x2);

      _mm_store_ps(&c[j], x2);
    }
    
}

void muladdc_sse(float *a, float *b, float *c, int Len, float *tc)
{
	int j;
	__m128 x0, x1, x2, t0, t1, m, add;
	//cout<<"tc="<<(long long int)tc % 16<<endl;
	add=_mm_set_ps(0,0,0,0);
	m = _mm_set_ps(-1, 1, -1, 1);
    for (j = 0; j < Len*2; j+=4) {	
      x0 = _mm_load_ps(&a[j]);
     // showm128(x0,"x0");
      t0 = _mm_load_ps(&b[j]);
    //  showm128(t0,"t0");
      t1 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(3, 3, 1, 1));
      t0 = _mm_shuffle_ps(t0, t0, _MM_SHUFFLE(2, 2, 0, 0));
      t1 = _mm_mul_ps(t1, m);
      x1 = _mm_mul_ps(x0, t0);
      x2 = _mm_mul_ps(x0, t1);

      x2 = _mm_shuffle_ps(x2, x2, _MM_SHUFFLE(2, 3, 0, 1));
      x2 = _mm_add_ps(x1, x2);
	 // showm128(x2,"x2");
	  add= _mm_add_ps(add,x2);	
	//  showm128(add,"add");
      
    }
    //showm128(add,"add");
    _mm_store_ps(&tc[0], add);
    c[0]=tc[0]+tc[2];
    c[1]=tc[1]+tc[3];
    
}

void *malloc_with_16bytes_alignment(size_t size)
{
    // 超量申请内存,多申请16个字节
    unsigned char *rawblock = (unsigned char *)malloc(size + 0x10);
    if (rawblock == NULL)
    {
        return NULL;
    }
    else if ((long)rawblock & 0xF)
    {
        // 得到对齐后的地址
        unsigned char *p = (unsigned char *)(((long)rawblock + 0xF) & (~0xF));
        // 用来得到原始地址的cookie
        p[-1] = p - rawblock;

        return p;
    }
    else
    {
        // 得到对齐后的地址
        unsigned char *p = rawblock + 0x10;
        // 用来得到原始地址的cookie
        p[-1] = p - rawblock;

        return p;
    }
}


void free_with_16bytes_alignment(void *memblock)
{
    if (memblock != NULL)
    {
        // 从cookie得到要释放的原始地址
        free((unsigned char *)memblock - *((unsigned char *)memblock - 1));
    }
}

Uint4 graycode(Uint4 data,Uint4 width)
{
	BYTE B[width];
	BYTE G[width];
	//cout<<data<<":";
	for(int i=1;i<=width;i++){
		B[i-1]=(data>>(width-i)) & 1;
	//	cout<<(int)B[i-1]<<" ";
	}
	//cout<<" | ";
	Uint4 graydata=0;
	for(int i=0;i<width;i++){
		if(i==0) G[i]=B[i];
		else
			G[i]=B[i] ^ B[i-1];
		//cout<<(int)G[i]<<" ";
		graydata=(graydata<<1)+G[i];
	}
	//cout<<"==>"<<graydata<<endl;
	return graydata;
}
