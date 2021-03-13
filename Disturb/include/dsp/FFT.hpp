#ifndef ASP_FFT_H
#define ASP_FFT_H

#include <complex>
#include <vector>
#include <fftw3.h>
#include <cstdlib>

namespace asp {

    typedef std::complex<double> Cmpd;
    typedef std::complex<float> Cmpf;
    typedef std::complex<int> Cmpi;
//typedef std::complex<uint8_t> Cmpb;
    typedef std::complex<short> Cmps;
    typedef std::complex<int8_t> Cmp8;

    typedef std::vector<Cmpd> CDSignal; //双精度复数信号
    typedef std::vector<Cmpf> CFSignal;
    typedef std::vector<Cmpi> CISignal;

    typedef std::vector<double> RDSignal;

    class FFT {
    protected:
        Cmpd *_in;  //fft input
        Cmpd *_out; //fft ouput
        fftw_plan _fftPlan;   //fft way
        unsigned int _fftSize;//fft size
        bool _maked;
        std::vector<double> _winWeights;
    public:
        FFT();

        ~FFT();

        void make(unsigned mfftsize, int ws);//construct fft
        void free();// release in, out and fftplan

        //实时输入
        template<typename datatype>
        void inputR(datatype *buf) {
            if (_maked == false) return;
            for (unsigned int i = 0; i < _fftSize; i++) {
                _in[i] = Cmpd(buf[i], 0.0);
            }
        }

        //复数输入
        template<typename datatype>
        void inputC(datatype *buf) {
            if (_maked == false) return;
            for (unsigned int i = 0; i < _fftSize; i++) {
                _in[i] = buf[i];
            }
        }


        void win();

        void excute();


        bool maked() { return _maked; }

        Cmpd *output() { return _out; };
    };


}
#endif
