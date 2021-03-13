#ifndef SIGSOURCE_H
#define SIGSOURCE_H
 
#include <stdio.h>
#include <unistd.h>     
#include <iostream>
    
#include <common/UDPSocket.hpp>
//#include <boost/program_options.hpp>
#include <media/AlsaSink.h>
#include <analog/FmModuler.h>
#include <analog/AmModuler.h>
#include <dsp/DigitalMod.h>
#include <dsp/DataSource.h>
#include <dsp/PrbsGenerator.h>
#include <device/HackrfDevice.hpp>

using namespace std;
//using namespace uhd;
//using namespace boost::gregorian;

typedef struct
{
	double freq;
	double db;
}TAdjust;

typedef struct
{
	double freq;
	double offset;	
}TFreqAdjust;



class SigSource 
{ 
enum SigSourceState {IDLE=0,BEGIN,SEND,END,ERROR};
enum ControlSignal {START=0,STOP};
protected:

	SigSourceState 	sstate; 
	ControlSignal		ctlsig;
	
	Alsa_sink          *audio;
    FmModuler			*fm;//FM 调制
    AmModuler           *am;//AM 调制
    DigitalMod          *dMod;//数字调制
    DigPram				dPram;//数字调制的主要参数
    data_source         *dsource; 
	UDPSocket 			*udp;
	HackrfDeviceTx      *hackrf;

	uint32_t         	masterIP;  //软件端IP地址
	int					masterPort;
 
	unsigned char    	sysstate; //系统状态  
	char				serialID[32]; //设备序列号
	char 				adjustfilename[32];//校准文件文件名
	BYTE				workmode; //设备工作模式  
	double 			center_freq;
	int					freq_index;
	BYTE				modulation; //0 sin, 1 FM, 2 AM, 3 GAUSS, 4 DIGTAL
	BYTE				band;
	int					gain;  
	 
	 
	short				*sigbuf;
	int					siglen;
  
	size_t				framesize;
	size_t				interp;
	int					modbits;
  
	char				wavfilename[256];
	FILE 				*fpwav;  //for am or fm source
	int					filestate; //0 close 1 open 2 error
	
	double				*freq_list;
	TAdjust 			*ad_list;
	TFreqAdjust        *freq_adjust;
	double     		CurAdj;
	
	int					list_pos;
	
	int 				flist_size;	
	int 				adlist_size;
	int					fad_size;
	double				sigamp;  
	Thread				work_thread;
public:
	SigSource(UDPSocket *wudp,HackrfDeviceTx *mhackrf);
	~SigSource();


	void cope_command(unsigned int cID, void *buf,unsigned int nbits,uint32_t sIP);
	void cope_connect(void *buf, unsigned int nbits, uint32_t sIP);
	void cope_set_parameter(void *buf,size_t nbits);
	void cope_ask_parameter(void *buf,size_t nbits);
	void cope_list_data(void *buf,size_t nbits);
	void cope_start(void *buf,size_t nbits);
	void cope_stop(void *buf,size_t nbits);
	void cope_ask_file(void *buf,size_t nbits);
	void send_connect_back(BYTE back,unsigned short ref);
	void send_parameter(BYTE cID,unsigned short ref);
	void send_start_ok(unsigned short ref);
    void send_stop_ok(unsigned short ref);
    void send_data_ack(unsigned short ref, BYTE list_type,int neednum);
	void send_file_data(char *askfilename,BYTE *filedata,int spos, int slen,unsigned short ref);
	
	void add_sig();
	void sub_sig();
	void generate_signal();
	
	void generate_sin();
	void generate_am();
	void generate_fm();
	void generate_gauss();
	void generate_digital();
	
	void play_signal();
	//int  play_signal(hackrf_transfer* transfer);
	
	void reset_sig();
 
	void enter_idle();
	void enter_begin();
	void enter_send();
	void enter_end();
	
	void start();	  
	void stop();
	 
 	void mainloop();
	void driveSigSource();
	void showparameter();
	
	
	void load_freq_list();
	void load_adjust_list();
	void load_freq_adjust();
	
	void save_list(BYTE list_type);
	double calinsertvalue(double infreq);
	double calinsertoffset(double infreq);
	
	void init(); 
	friend void *work_prog(SigSource*);

};
void *work_prog(SigSource*);


#endif
