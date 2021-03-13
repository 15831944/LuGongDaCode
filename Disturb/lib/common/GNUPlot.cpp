#include <common/GNUPlot.h>
using namespace std;
GNUPlot::GNUPlot() throw(string) {
	gnuplotpipe=popen("gnuplot ","w");
	if (!gnuplotpipe) {
	throw("Gnuplot not found !");
}
}    
 
GNUPlot::~GNUPlot() {
	fprintf(gnuplotpipe,"exit\n");
	pclose(gnuplotpipe);
}

void GNUPlot::operator()(const string & command) {
	fprintf(gnuplotpipe,"%s\n",command.c_str());
	fflush(gnuplotpipe);
	// flush is necessary, nothing gets plotted else
};

void GNUPlot::plotter(const string & command) {
	fprintf(gnuplotpipe,"%s\n",command.c_str());
	fflush(gnuplotpipe);
	// flush is necessary, nothing gets plotted else
};
void GNUPlot::plot(float *x, float *y, int number)
{

	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%f %f\n",x[i],y[i]);
	}	
	fclose(freal);
	plotter("plot \".real.txt\" w l");
}


void GNUPlot::plot(int *x, int *y, int number)
{

	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i, x[i],y[i]);
	}	
	fclose(freal);
	plotter("plot \".real.txt\" w l");
}
void GNUPlot::plot(complex16 *buf, int number)
{
	FILE *freal,*fimag;
	freal=fopen(".real.txt","w");
	fimag=fopen(".imag.txt","w");
        

	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i,buf[i].real());
		fprintf(fimag,"%d %d\n",i,buf[i].imag());
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
	
}

void GNUPlot::plot(complex32 *buf, int number)
{
	FILE *freal,*fimag;
	freal=fopen(".real.txt","w");
	fimag=fopen(".imag.txt","w");
        
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i,buf[i].real());
		fprintf(fimag,"%d %d\n",i,buf[i].imag());
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
	
}

void GNUPlot::plot(std::vector<fcomplex> vec)
{
	FILE *freal,*fimag;
	freal=fopen(".real.txt","w");
	fimag=fopen(".imag.txt","w");       
	for(int i=0;i<vec.size();i++){
		fprintf(freal,"%d %f\n",i,vec[i].real());
		fprintf(fimag,"%d %f\n",i,vec[i].imag());
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
	
	
}

void GNUPlot::plotstar(std::vector<fcomplex> vec)
{
	FILE *freal;
	freal=fopen(".real.txt","w");    
	for(int i=0;i<vec.size();i++){
		fprintf(freal,"%f %f\n",vec[i].real(),vec[i].imag());
	}
	fprintf(freal,"%f %f\n",1.5,1.5);
	fprintf(freal,"%f %f\n",1.5,-1.5);
	fprintf(freal,"%f %f\n",-1.5,1.5);
	fprintf(freal,"%f %f\n",-1.5,-1.5);
	fclose(freal);
	

	plotter("plot \".real.txt\" ");

}
void GNUPlot::plotstar(fcomplex *buf, int number)
{
	FILE *freal;
	freal=fopen(".real.txt","w");    
	for(int i=0;i<number;i++){
		fprintf(freal,"%f %f\n",buf[i].real(),buf[i].imag());
	}
	
	fclose(freal);
	

	plotter("plot \".real.txt\" ");

}

void GNUPlot::plotabs(fcomplex *buf, int number)
{
	FILE *freal;
	freal=fopen(".abs.txt","w");    
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %f\n",i,sqrt(buf[i].norm2()));
	}
	
	fclose(freal);
	

	plotter("plot \".abs.txt\" w l");

}

void GNUPlot::plotabs(complex8 *buf, int number)
{
	FILE *freal;
	freal=fopen(".abs.txt","w");    
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %f\n",i,sqrt(buf[i].norm2()));
	}
	
	fclose(freal);
	

	plotter("plot \".abs.txt\" w l");

}

void GNUPlot::plotabs(dcomplex *buf, int number)
{
	FILE *freal;
	freal=fopen(".abs.txt","w");
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %lf\n",i,sqrt(buf[i].norm2()));
	}

	fclose(freal);


	plotter("plot \".abs.txt\" w l");

}
void GNUPlot::plot(fcomplex *buf, int number)
{
	FILE *freal,*fimag;
	freal=fopen(".real.txt","w");
	fimag=fopen(".imag.txt","w");       
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %f\n",i,buf[i].real());
		fprintf(fimag,"%d %f\n",i,buf[i].imag());
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
	
	
}

void GNUPlot::plotreal(fcomplex *buf, int number)
{
	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %f\n",i,buf[i].real());
	}
	
	fclose(freal);	

	plotter("plot \".real.txt\" w l");
	
	
}
void GNUPlot::plot(short *buf, int number)
{
	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i,buf[i]);
	}
	
	fclose(freal);
	plotter("plot \".real.txt\" w l");

	
}
void GNUPlot::plot(int *buf, int number)
{

	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i,buf[i]);
	}
	
	fclose(freal);
	plotter("plot \".real.txt\" w l");

	
}
void GNUPlot::plot(BYTE *buf, int number)
{

	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %d\n",i,buf[i]);
	}
	
	fclose(freal);
	plotter("plot \".real.txt\" w l");

	
}

void GNUPlot::plot(float *buf, int number)
{

	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<number;i++){
		fprintf(freal,"%d %f\n",i,buf[i]);
	}
	
	fclose(freal);
	plotter("plot \".real.txt\" w l");

	
	
	
}

void GNUPlot::plot(vector<float> vec)
{
	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<vec.size();i++){
		float elm=vec[i];
	//	cout<<elm<<endl;
		fprintf(freal,"%d %f\n",i,elm);
	}
	
	fclose(freal);
	plotter("plot \".real.txt\" w l");
	
}

void GNUPlot::plot(vector<BYTE> vec)
{
	FILE *freal;
	freal=fopen(".real.txt","w");   
	for(int i=0;i<vec.size();i++){
		int elm=vec[i];
	//	cout<<elm<<endl;
		fprintf(freal,"%d %d\n",i,elm);
	}
	
	fclose(freal);
	plotter("plot \"..real.txt\" w l");
	
}

void GNUPlot::plot(std::vector<short> vec) {
    FILE *freal;
    freal=fopen(".real.txt","w");
    for(int i=0;i<vec.size();i++){
        auto elm=vec[i];
//        	cout<<elm<<endl;
        fprintf(freal,"%d %d\n",i,elm);
    }

    fclose(freal);
    plotter("plot \".real.txt\" w l");
}

