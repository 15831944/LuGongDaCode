#include <dsp/FFT.hpp>
#include <assert.h>
#include <iostream>
#include <vector>

namespace asp {

    FFT::FFT() {
        _maked = false;
    };

    FFT::~FFT() {
        free();
    };


    std::vector<double>
    windesign(int type, int ntaps) {
        std::vector<double> taps(ntaps);
        int M = ntaps - 1;        // filter order

        switch (type) {
            case 0:
                for (int n = 0; n < ntaps; n++)
                    taps[n] = 1;
                break;

            case 2:
                for (int n = 0; n < ntaps; n++)
                    taps[n] = 0.54 - 0.46 * cos((2 * M_PI * n) / M);
                break;

            case 1:
                for (int n = 0; n < ntaps; n++)
                    taps[n] = 0.5 - 0.5 * cos((2 * M_PI * n) / M);
                break;

            case 3:
                for (int n = 0; n < ntaps; n++)
                    taps[n] = 0.42 - 0.50 * cos((2 * M_PI * n) / (M - 1)) - 0.08 * cos((4 * M_PI * n) / (M - 1));
                break;
        }

        return taps;
    }


    void FFT::make(unsigned fftSize, int ws) {
        assert(_maked == false);
        _fftSize = fftSize;

        _in = (Cmpd *) fftw_malloc(sizeof(fftw_complex) * _fftSize);
        _out = (Cmpd *) fftw_malloc(sizeof(fftw_complex) * _fftSize);
        _fftPlan = fftw_plan_dft_1d(fftSize,
                                    reinterpret_cast<fftw_complex *>(_in),
                                    reinterpret_cast<fftw_complex *>(_out),
                                    FFTW_FORWARD,
                                    FFTW_ESTIMATE);
        _winWeights = windesign(ws, _fftSize);

        _maked = true;
    }

    void FFT::free() {
        if (_maked) {
            fftw_destroy_plan(_fftPlan);
            fftw_free(_in);
            fftw_free(_out);
            _maked = false;
        }

    }


    void FFT::excute() {
        assert(_maked == true);
        //win();
        fftw_execute(_fftPlan);
    }

    void FFT::win() {
        for (unsigned int i = 0; i < _fftSize; i++) {
            _in[i] = Cmpd(_in[i].real() * _winWeights[i], _in[i].imag() * _winWeights[i]);
        }
    }

}

