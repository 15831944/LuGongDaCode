#include <dsp/DFT.h>

#include <assert.h>
#include <iostream>
#include <vector>
using namespace std;
dfft::dfft()
{
	mstate=FREE;
};

dfft::~dfft()
{
	free();
};

vector<float>windesign(int type, int ntaps)  //构造了一个包含type个值为ntaps的元素的Vector windesign
{
	vector<float> taps(ntaps);//声明一个存放ntaps个float类型的vector taps
	int	N= ntaps - 1;		// filter order

	switch (type)
	{
	case 0:
			for (int n = 0; n < ntaps; n++)
				taps[n] = 1;
			break;

	case 2:
			for (int n = 0; n < ntaps; n++)
				taps[n] = 0.54 - 0.46 * cos ((2 * M_PI * n) / N);//M_PI在math.h中有如下定义M_PI#define M_PI 3.14159265358979323846
			break;

	case 1:
			for (int n = 0; n < ntaps; n++)
				taps[n] = 0.5 - 0.5 * cos ((2 * M_PI * n) / N);
			break;

	case 3:
			for (int n = 0; n < ntaps; n++)
				taps[n] = 0.42 - 0.50 * cos ((2*M_PI * n) / (N-1)) - 0.08 * cos ((4*M_PI * n) / (N-1));
			break;
	}
	return taps;
}

void dfft::make(unsigned mfftsize,int ws)  //hwin=new double [mfftsize]; hwin[n]=taps[n];
											//vector<float> taps=windesign(ws, mfftsize);
{
	assert(mstate==FREE);// assert 测试一个条件并可能使程序终止
	fftsize=mfftsize;
	//先用fftw_malloc分配输入输出内存，然后输入数据赋值，然后创建变换方案（fftw_plan），然后执行变换（fftw_execute），最后释放资源
	in = (dcomplex*) fftw_malloc(sizeof(fftw_complex) * fftsize);
	out =(dcomplex*) fftw_malloc(sizeof(fftw_complex) * fftsize);
	///函数接口
	fftplan = fftw_plan_dft_1d(fftsize, 
                             reinterpret_cast<fftw_complex *>(in), //reinterpret_cast强制转换类型符
                             reinterpret_cast<fftw_complex *>(out), 
                             FFTW_FORWARD, //正变换，逆变换为FFTW_BACKWARD;//sign
                             FFTW_ESTIMATE);//flags
                             
	hwin=new double [mfftsize];
	vector<float> taps=windesign(ws, mfftsize); // 由上 vector<float> taps(ntaps)
	for (int n = 0; n < mfftsize; n++) 
	{
		hwin[n]=taps[n];
	}
	mstate=MAKE;
}

void dfft::free()
{
	if(mstate==MAKE)
	{
		fftw_destroy_plan(fftplan);
		fftw_free(in); 
		fftw_free(out);
		delete (double *) hwin;
		mstate=FREE;
	}
 
}

void dfft::input(short *buf) //2字节16位
{
	assert(mstate==MAKE);
	for (unsigned int i=0;i<fftsize;i++)
	{
		in[i]=Complex <double> (buf[2*i],buf[2*i+1]); 
	}
}

void dfft::input(int8_t *buf)//1字节8位
{
	assert(mstate==MAKE);
	for (unsigned int i=0;i<fftsize;i++)
	{
		in[i]=Complex <double> (buf[2*i],buf[2*i+1]); 
	}
}

void dfft::input(fcomplex *buf) //4字节32位
{
	assert(mstate==MAKE);
	for (unsigned int i=0;i<fftsize;i++)
	{
	in[i]=dcomplex(buf[i].real(),buf[i].imag()); 
	}
}

void dfft::excute()
{
  assert(mstate==MAKE);
  fftw_execute(fftplan);  
} 

void dfft::win() //构造矩形窗
{
	for (unsigned int i=0;i<fftsize;i++)
	{
		in[i]=dcomplex(in[i].r*hwin[i],in[i].i*hwin[i]);
	}
	
}


