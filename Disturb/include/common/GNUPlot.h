#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <vector>

#include <common/Complex.hpp>
typedef unsigned char BYTE;
class GNUPlot {
protected:
	FILE *gnuplotpipe;
public:
	//create and interface
	GNUPlot() throw(std::string);
	~GNUPlot();
	void operator ()(const std::string & command);
	void plotter(const std::string &command);

	
	
	//user application
	void plot(float *x, float *y, int number);
	void plot(int *x, int *y, int number);
	void plot(complex16 *buf, int number);
	void plot(complex32 *buf, int number);
	void plot(fcomplex *buf, int number);
	void plotreal(fcomplex *buf, int number);
	
	
	
	void plot(std::vector<fcomplex> vec);
	
	void plot(short *buf, int number);
	void plot(int *buf, int number);
	void plot(BYTE *buf, int number);
	void plot(float *buf, int number);
	void plot(std::vector<float> vec);
    void plot(std::vector<short> vec);
	
	
	void plot(std::vector<BYTE> vec);
	
	void plotstar(std::vector<fcomplex> vec);
	void plotstar(fcomplex *buf, int number);
	void plotabs(fcomplex *buf,int number);
	void plotabs(complex8 *buf, int number);
	void plotabs(dcomplex *buf, int number);

};


#endif
