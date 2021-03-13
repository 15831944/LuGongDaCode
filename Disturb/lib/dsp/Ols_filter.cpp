#include <dsp/Ols_filter.hpp>
#include <iostream>
#include <memory.h>
#include <stdlib.h>


#define MAX_MEM_SIZE 65536
namespace asp {

    Ols_filter::Ols_filter(CDSignal &taps) {
        _M = taps.size();
        _N = GetOptimalFFTSize(_M);
        _VL = _N - _M + 1;

        std::cout << "M=" << _M << " N=" << _N << " VL=" << _VL << std::endl;


        _mem = new Cmp8[MAX_MEM_SIZE];
        _pos = 0;

        bzero(_mem, sizeof(Cmp8) * MAX_MEM_SIZE);


        CDSignal h; ///系数
        for (int i = 0; i < _N; i++) {
            if (i < _M)
                h.push_back(taps[i]);
            else
                h.push_back(0);
        }
        _fft = new FFT();
        _fft->make(_N, 0);


        ///计算hjw, 初始化 xjw,yjw
        _fft->inputC(&h[0]);
        _fft->excute();
        Cmpd *out = _fft->output();
        for (int i = 0; i < _N; i++) {
            _hjw.push_back(std::conj(out[i]));
            _xjw.push_back(0);
            _yjw.push_back(0);
        }







        //for(auto i=0;i<FFTSize;i++) std::cout<<out[i]<<std::endl;

    }

    Ols_filter::~ Ols_filter() {

        delete (FFT *) _fft;
        delete[] _mem;

    }


    ///complex short 型数据输入,适配USRP的数据类型
    void Ols_filter::Work(Cmp8 *input, int L, CDSignal &output) {
        //output.clear(); //清空输出向量

        int neededCopeLen = L + _pos;

        if (neededCopeLen > MAX_MEM_SIZE) {
            std::cout << " the size of input data (" << neededCopeLen << ") is bigger than that of mem  ("
                      << MAX_MEM_SIZE << ")" << std::endl;
            std::cout << " the size of _pos (" << _pos << std::endl;
            exit(0);
        }

        memcpy(&_mem[_pos], input, L * sizeof(Cmp8)); //拷贝数据

        int rb = 0;
        while (neededCopeLen >= _N) {

            _fft->inputC<Cmp8>(&_mem[rb]);
            _fft->excute();
            Cmpd *out = _fft->output();
            for (int i = 0; i < _N; i++) {
                _xjw[i] = out[i]; ///不用memcpy 是担心 out的内存结构和 xjw 不一样
                _yjw[i] = conj(_xjw[i] * _hjw[i]);
            }

            ///fft 实现 ifft conj(fft(conj(yjw)))=ifft(yjw)
            _fft->inputC(&_yjw[0]);
            _fft->excute();
            out = _fft->output();

            for (int i = 0; i < _VL; i++) {
                Cmpd odata = Cmpd(out[i].real() / _N, -out[i].imag() / _N);
                output.push_back(odata);
            }


            //带有一定的重叠 (N-VL)
            rb = rb + _VL;
            neededCopeLen = neededCopeLen - _VL;
        }

        if (neededCopeLen > 0) {
            memcpy(&_mem[0], &_mem[rb], neededCopeLen * sizeof(Cmp8));
            _pos = neededCopeLen;
        }


    }


    int Ols_filter::GetOptimalFFTSize(int M) {
        double MinO = 1e100; ///表示无穷大
        int Size;
        for (int i = 1; i < 10; i++) {
            int N = i * M;
            double p = log2(N);
            if (p == floor(p))
                N = pow(2, floor(p));
            else
                N = pow(2, floor(p) + 1);
            double O = N * (log2(N) + 1) / (N - M + 1);
            if (O < MinO) {
                MinO = O;
                Size = N;
            }
        }
        double rate = Size * (log2(Size) + 1) / (Size - M + 1) / M;
        std::cout<<"rate="<<rate<<std::endl;
        return Size;
    }


}
