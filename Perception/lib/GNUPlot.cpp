#include "GNUPlot.hpp"

using namespace std;
using namespace terjin;

GNUPlot::GNUPlot() noexcept(false) {
#ifdef _MSVC_LANG

#else
    gnuplotpipe = popen("gnuplot ", "w");
    if (!gnuplotpipe) {
        throw ("Gnuplot not found !");
    }
#endif
}

GNUPlot::~GNUPlot() {
#ifdef _MSVC_LANG

#else
    cout << "gnuplot 析构" << endl;

    fprintf(gnuplotpipe, "exit\n");
    pclose(gnuplotpipe);
#endif
}

void GNUPlot::operator()(const string &command) {
#ifdef _MSVC_LANG

#else
    fprintf(gnuplotpipe, "%s\n", command.c_str());
    fflush(gnuplotpipe);
    // flush is necessary, nothing gets plotted else
#endif
};

void GNUPlot::plotter(const string &command) {
#ifdef _MSVC_LANG

#else
	fprintf(gnuplotpipe, "%s\n", command.c_str());
    fflush(gnuplotpipe);
    // flush is necessary, nothing gets plotted else
#endif
};

void GNUPlot::plot(float *x, float *y, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%f %f\n", x[i], y[i]);
    }
    fclose(freal);
    plotter("plot \".real.txt\" w l");
#endif
}

void GNUPlot::plot(std::complex<short> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal, *fimag;
    freal = fopen(".real.txt", "w");
    fimag = fopen(".imag.txt", "w");

    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %d\n", i, buf[i].real());
        fprintf(fimag, "%d %d\n", i, buf[i].imag());
    }

    fclose(freal);
    fclose(fimag);

    plotter("set multiplot");//   %设置为多图模式
    plotter("set origin 0.0,0.0"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".real.txt\" w l");
    plotter("set origin 0.0,0.5"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".imag.txt\" w l");
#endif

}

void GNUPlot::plot(std::complex<long> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal, *fimag;
    freal = fopen(".real.txt", "w");
    fimag = fopen(".imag.txt", "w");

    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %d\n", i, buf[i].real());
        fprintf(fimag, "%d %d\n", i, buf[i].imag());
    }

    fclose(freal);
    fclose(fimag);

    plotter("set multiplot");//   %设置为多图模式
    plotter("set origin 0.0,0.0"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".real.txt\" w l");
    plotter("set origin 0.0,0.5"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".imag.txt\" w l");
#endif
}

void GNUPlot::plot(std::vector<std::complex<float>> vec) {
#ifdef _MSVC_LANG

#else
	FILE *freal, *fimag;
    freal = fopen(".real.txt", "w");
    fimag = fopen(".imag.txt", "w");
    for (int i = 0; i < vec.size(); i++) {
        fprintf(freal, "%d %f\n", i, vec[i].real());
        fprintf(fimag, "%d %f\n", i, vec[i].imag());
    }

    fclose(freal);
    fclose(fimag);

    plotter("set multiplot");//   %设置为多图模式
    plotter("set origin 0.0,0.5"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".real.txt\" w l");
    plotter("set origin 0.0,0.0"); //  %设置第二个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    //plotter("set yrange[-2:2]");
    plotter("plot \".imag.txt\" w l");

#endif
}

void GNUPlot::plotstar(std::vector<std::complex<float>> vec) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < vec.size(); i++) {
        fprintf(freal, "%f %f\n", vec[i].real(), vec[i].imag());
    }
    fprintf(freal, "%f %f\n", 1.5, 1.5);
    fprintf(freal, "%f %f\n", 1.5, -1.5);
    fprintf(freal, "%f %f\n", -1.5, 1.5);
    fprintf(freal, "%f %f\n", -1.5, -1.5);
    fclose(freal);


    plotter("plot \".real.txt\" ");
#endif
}

void GNUPlot::plotstar(std::complex<float> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%f %f\n", buf[i].real(), buf[i].imag());
    }

    fclose(freal);


    plotter("plot \".real.txt\" ");
#endif
}

void GNUPlot::plotabs(std::complex<float> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".abs.txt", "w");
    for (int i = 0; i < number; i++) {
//		fprintf(freal,"%d %f\n",i,sqrt(buf[i].norm2()));
        fprintf(freal, "%d %f\n", i, sqrt(std::norm(buf[i])));
    }

    fclose(freal);


    plotter("plot \".abs.txt\" w l");
#endif
}

void GNUPlot::plot(std::complex<float> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal, *fimag;
    freal = fopen(".real.txt", "w");
    fimag = fopen(".imag.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %f\n", i, buf[i].real());
        fprintf(fimag, "%d %f\n", i, buf[i].imag());
    }

    fclose(freal);
    fclose(fimag);

    plotter("set multiplot");//   %设置为多图模式
    plotter("set origin 0.0,0.5"); //  %设置第一个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    plotter("plot \".real.txt\" w l");
    plotter("set origin 0.0,0.0"); //  %设置第二个图的原点的位置
    plotter("set size 1,0.5");//  %设置第一个图的大小
    //plotter("set yrange[-2:2]");
    plotter("plot \".imag.txt\" w l");

#endif
}

void GNUPlot::plotreal(std::complex<float> *buf, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %f\n", i, buf[i].real());
    }

    fclose(freal);

    plotter("plot \".real.txt\" w l");
#endif

}

void GNUPlot::plot(short *buf, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %d\n", i, buf[i]);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");
#endif

}

void GNUPlot::plot(int *buf, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %d\n", i, buf[i]);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");
#endif

}

void GNUPlot::plot(double *buf, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %f\n", i, buf[i]);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");

#endif
}

void GNUPlot::plot(BYTE *buf, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %d\n", i, buf[i]);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");

#endif
}

void GNUPlot::plot(float *buf, int number) {
#ifdef _MSVC_LANG

#else
    FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%d %f\n", i, buf[i]);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");

#endif
}

void GNUPlot::plot(vector<float> vec) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < vec.size(); i++) {
        float elm = vec[i];
        //	cout<<elm<<endl;
        fprintf(freal, "%d %f\n", i, elm);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");
#endif
}

void GNUPlot::plot(vector<BYTE> vec) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < vec.size(); i++) {
        int elm = vec[i];
        //	cout<<elm<<endl;
        fprintf(freal, "%d %d\n", i, elm);
    }

    fclose(freal);
    plotter("plot \"..real.txt\" w l");
#endif
}

void GNUPlot::plot(double *x, double *y, int number) {
#ifdef _MSVC_LANG

#else
	FILE *freal;
    freal = fopen(".real.txt", "w");
    for (int i = 0; i < number; i++) {
        fprintf(freal, "%f %f\n", x[i], y[i]);
    }
    fclose(freal);
    plotter("plot \".real.txt\" w l");
#endif
}

