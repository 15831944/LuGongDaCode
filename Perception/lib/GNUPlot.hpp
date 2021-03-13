#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <complex>

#include <terjin/config.h>
#include <terjin/utils/GNUPlotIface.hpp>
//typedef std::complex<short> std::complex<short>;
//typedef std::complex<long> std::complex<long>;
//typedef std::complex<float> std::complex<float>;
//typedef std::complex<double> std::complex<double>;

typedef unsigned char BYTE;

namespace terjin {

    class TERJIN_API GNUPlot :public GNUPlotIface{
    protected:
        FILE *gnuplotpipe;
    public:
        //create and interface
        GNUPlot() noexcept(false);

        ~GNUPlot();

        void operator()(const std::string &command);

        void plotter(const std::string &command);


        //user application
        void plot(float *x, float *y, int number);

        void plot(std::complex<short> *buf, int number);

        void plot(std::complex<long> *buf, int number);

        void plot(std::complex<float> *buf, int number);

        void plotreal(std::complex<float> *buf, int number);

        void plot(double *x,double *y,int number);


        void plot(std::vector<std::complex<float>> vec);

        void plot(short *buf, int number);

        void plot(int *buf, int number);

        void plot(double *buf,int number);

        void plot(BYTE *buf, int number);

        void plot(float *buf, int number);

        void plot(std::vector<float> vec);


        void plot(std::vector<BYTE> vec);

        void plotstar(std::vector<std::complex<float>> vec);

        void plotstar(std::complex<float> *buf, int number);

        void plotabs(std::complex<float> *buf, int number);

    };
}


#endif
