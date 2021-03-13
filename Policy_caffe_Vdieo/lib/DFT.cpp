#include "DFT.hpp"
#include <cassert>
#include <iostream>
#include <vector>

#ifdef ENABLE_MKL

#include <fftw/fftw3.h>

#else
#include <fftw3.h>
#endif

#ifndef M_PI
#define M_PI 3.141592654
#endif


using namespace std;
using namespace terjin;


class DFTImpl : public DFT {
public:
    typedef boost::shared_ptr<DFTImpl> sptr;

private:
    std::complex<double> *in{};  //fft input
    std::complex<double> *out{}; //fft ouput
    std::complex<double> *out_temp{};//temp
    fftw_plan _fft_plan{};   //fft way
    unsigned int _fft_size{};//fft size
    fft_make_status _make_state;  //a sign of the state of DFTImpl module.
    double *_h_win{};
public:
    DFTImpl();

    ~DFTImpl();

    void make(unsigned fft_size, int ws, bool ifft) override;//construct fft

    void free() override;// release in, out and fftplan

    void input(short *buf) override;

    void inputRotate(short *buf, double theta0, double dtheta) override;

    void inputRotate(std::complex<int16_t> *buf, double theta0, double dtheta) override;

    void input(int8_t *buf) override;

    void input(std::complex<short> *buf) override;

    void input(std::complex<float> *buf) override;

    void input(std::complex<double> *buf) override;

    void input(double *buf) override;

    void input(std::vector<float> &buf) override;

    void win() override;

    void excute() override;

    void shift() override;

    void excuteTest() override;

    fft_make_status readfftmstate() override { return _make_state; }

    std::complex<double> *output() override { return out; };
};


DFTImpl::DFTImpl() {
    _make_state = FREE;
};

DFTImpl::~DFTImpl() {
    free();
};

vector<double> win_design(int type, size_t n_taps) {
    vector<double> taps(n_taps);
    size_t M = n_taps - 1;        // filter order

    switch (type) {
        case 0:
            for (size_t n = 0; n < n_taps; n++)
                taps[n] = 1.0;
            break;
        case 2:
            for (size_t n = 0; n < n_taps; n++)
                taps[n] = 0.54 - 0.46 * cos((2 * M_PI * n) / M);
            break;
        case 1:
            for (size_t n = 0; n < n_taps; n++)
                taps[n] = 0.5 - 0.5 * cos((2 * M_PI * n) / M);
            break;
        case 3:
            for (size_t n = 0; n < n_taps; n++)
                taps[n] = 0.42 - 0.50 * cos((2 * M_PI * n) / (M - 1)) - 0.08 * cos((4 * M_PI * n) / (M - 1));
            break;
        default: {
            throw std::runtime_error("unknown win type type");
        }
    }

    return taps;
}

void DFTImpl::make(unsigned fft_size, int ws, bool ifft) {
    assert(_make_state == FREE);
    _fft_size = fft_size;
    in = (std::complex<double> *) fftw_malloc(sizeof(fftw_complex) * _fft_size);
    out = (std::complex<double> *) fftw_malloc(sizeof(fftw_complex) * _fft_size);
    out_temp = (std::complex<double> *) fftw_malloc(sizeof(fftw_complex) * _fft_size);
    if (ifft) {
        _fft_plan = fftw_plan_dft_1d(_fft_size,
                                     reinterpret_cast<fftw_complex *>(in),
                                     reinterpret_cast<fftw_complex *>(out),
                                     FFTW_BACKWARD,
                                     FFTW_ESTIMATE);
    } else {
        _fft_plan = fftw_plan_dft_1d(_fft_size,
                                     reinterpret_cast<fftw_complex *>(in),
                                     reinterpret_cast<fftw_complex *>(out),
                                     FFTW_FORWARD,
                                     FFTW_ESTIMATE);
    }
    _h_win = new double[_fft_size];

    vector<double> taps = win_design(ws, _fft_size);
    for (size_t n = 0; n < _fft_size; n++) {
        _h_win[n] = taps[n];
    }
    _make_state = MAKE;
	BOOST_ASSERT(_fft_plan != nullptr);

}

void DFTImpl::free() {
    if (_make_state == MAKE) {
        fftw_destroy_plan(_fft_plan);
        fftw_free(in);
        fftw_free(out);
        fftw_free(out_temp);
        //delete (double *) _h_win;
        delete[] _h_win;
        _h_win = nullptr;
        _make_state = FREE;
    }
}

void DFTImpl::input(short *buf) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        in[i] = std::complex<double>(buf[2 * i], buf[2 * i + 1]);
    }
}

void DFTImpl::inputRotate(short *buf, double theta0, double dtheta) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        theta0 = theta0 + dtheta;
        in[i] = std::complex<double>(buf[2 * i], buf[2 * i + 1]);
        in[i] = in[i] * std::complex<double>(cos(theta0), sin(theta0));
    }
}

void DFTImpl::inputRotate(std::complex<int16_t> *buf, double theta0, double dtheta) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        theta0 = theta0 + dtheta;
        in[i] = std::complex<double>(buf[i].real(), buf[i].imag());
        in[i] = in[i] * std::complex<double>(cos(theta0), sin(theta0));
    }
}


void DFTImpl::input(int8_t *buf) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        in[i] = std::complex<double>(buf[2 * i], buf[2 * i + 1]);
    }
}


void DFTImpl::input(std::complex<float> *buf) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        in[i] = std::complex<double>(buf[i].real(), buf[i].imag());
    }
}

void DFTImpl::input(double *buf) {
    assert(_make_state == MAKE);
    for (size_t i = 0; i < _fft_size; ++i) {
        in[i] = std::complex<double>(buf[i], 0.0);
    }
}

void DFTImpl::input(std::complex<double> *buf) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        in[i] = buf[i];
    }
}

void DFTImpl::input(std::vector<float> &buf) {
    assert(_make_state == MAKE);
    for (unsigned int i = 0; i < _fft_size; i++) {
        if (i < buf.size())
            in[i] = buf[i];
        else
            in[i] = 0;
    }
}

void DFTImpl::input(std::complex<short> *buf) {
    assert(_make_state == MAKE);
    for (size_t i = 0; i < _fft_size; ++i) {
        in[i] = std::complex<double>(buf[i].real(), buf[i].imag());
    }
}


void DFTImpl::excuteTest() {
    for (unsigned int i = 0; i < _fft_size; i++) {
        out[i] = in[i];
    }
}

void DFTImpl::excute() {
    assert(_make_state == MAKE);
    win();
    fftw_execute(_fft_plan);
}

void DFTImpl::win() {
    for (unsigned int i = 0; i < _fft_size; i++) {
        in[i] = std::complex<double>(in[i].real() * _h_win[i], in[i].imag() * _h_win[i]);
    }
}

DFT::sptr DFT::make() {
    return DFT::sptr(new DFTImpl());
}

void DFTImpl::shift() {
    for (uint32_t i = 0; i < _fft_size; ++i) {
        if (i < _fft_size / 2) {
            out_temp[i] = out[i];
            out[i] = out[i + _fft_size / 2];
        } else {
            out[i] = out_temp[i - _fft_size / 2];
        }
    }
}



