#ifndef OLS_FILTER_H
#define OLS_FILTER_H

#include <dsp/FFT.hpp>

namespace asp {


    class Ols_filter {
    protected:
        int _pos; //存储器中已有数据的大小
        int _N; /// 填充后的长度
        int _M; ///滤波器系数的长度
        int _VL;  ///处理后有效的长度
        Cmp8 *_mem; ///缓存
        FFT *_fft;  ///fft模块
        CDSignal _hjw; ///系数频谱

        CDSignal _xjw; ///输入片段频谱
        CDSignal _yjw; ///输出片段频谱


    public:
        //taps 滤波器系数
        Ols_filter(CDSignal &taps);

        ~Ols_filter();

        ///complex short 型数据输入,适配USRP的数据类型
        ///input 输入
        /// L 输入的长度
        ///ouput输出向量
        void Work(Cmp8 *input, int L, CDSignal &output);


    private:
        int GetOptimalFFTSize(int M);


    };


}
#endif
