#ifndef TERJIN_DSP_DFT_H
#define TERJIN_DSP_DFT_H

#include <complex>
#include <vector>
#include <boost/shared_ptr.hpp>

constexpr double pi = 3.14159265358;
namespace terjin {


    class  DFT {
    public:
        typedef boost::shared_ptr<DFT> sptr;
        enum fft_make_status {
            MAKE, FREE
        };

    public:
        static sptr make();

        virtual ~DFT() = default;;

        virtual void make(unsigned fft_size, int ws = 0, bool ifft = false) = 0;//construct fft

        virtual void free() = 0;// release in, out and fftplan

        virtual void input(short *buf) = 0;

        virtual void inputRotate(short *buf, double theta0, double dtheta) = 0;

        virtual void inputRotate(std::complex<int16_t> *buf, double theta0, double dtheta) = 0;

        virtual void input(int8_t *buf) = 0;

        virtual void input(std::complex<short> *buf) = 0;

        virtual void input(std::complex<float> *buf) = 0;

        virtual void input(std::complex<double> *buf) = 0;

        virtual void input(double *buf) = 0;

        virtual void input(std::vector<float> &buf) = 0;

        virtual void win() = 0;

        virtual void excute() = 0;

        virtual void shift() = 0;

        virtual void excuteTest() = 0;

        virtual fft_make_status readfftmstate() = 0;

        virtual std::complex<double> *output() = 0;
    };

}
#endif
