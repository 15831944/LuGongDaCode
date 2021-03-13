/*
 * Copyright (c) 2011, meteor
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     *  Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *
 *     *  Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * SDSA 
 *
 * 	Software Defined Spectrum Analyze Device  
 * 	
 */
#ifndef COMMEASSAGE_H
#define COMMEASSAGE_H


#include <stdio.h>
#include <unistd.h>
#include <string.h>    
#include <iostream>    


#include <common/Thread.h>
#include <dsp/DigitalSig.h>

using namespace std;
enum SweepDirection {NODIRECTION,UP,DOWN};
enum WorkMode {NOMODE,FFM,FSCAN,DSCAN,FLS,DIGS,DIGM};//DIGS 表示星座图模式，DIGM 表示解调莫死
enum RunStop {RUN,STOP,CONTINUE};
enum Detector {PAV,PMAX,FAST};
enum Demodulation {AM,FM,OFF};
enum WaveOut {WOFF,SOFT,HARD};

const char sdirection[3][16]={"NODIRECTION","UP","DOWN"};
const char workmode[7][8]= {"NOMODE","FFM","FSCAN","DSCAN","FLS","DIGS","DIGM"};
const char runstop[4][16]={"RUN","STOP","CONTINUE"};
const char detector[3][8]={"PAV","PMAX","FAST"};
const char demodulation[3][8]={"AM","FM","OFF"};

class COMMessage //a class for storing the command
{
protected:
  Mutex             mlock;
  bool              isfinished;//
  int               type;
  RunStop           run_stop;
  unsigned int      sweepcount;
  WorkMode          mode;
  double            startfreq;
  double            stopfreq;
  double            stepfreq;
  SweepDirection    direction;
  Detector          detr;
  Demodulation      demod;
  double            dwell;
  bool              gpsopen;
  int		    	tune_delay;
  int               dwell_delay;
  float		    timezone;
  WaveOut			wout;
  DigPram			dPram;
  int 			gain;
  int			winstyle;


public:
  double            flist[1024];  //defined as public variable
  int               fnum;
  std::string		mboard_name;
  std::string		dboard_name;
  
  COMMessage(){isfinished=false;fnum=0;gpsopen=false;};
  ~COMMessage(){};  
  bool checkmessage(){return isfinished;};
  void changestate(){};
  void resest();
  void setmessage(){isfinished=true;};

  void setwaveout(WaveOut mout){mlock.lock();wout=mout;mlock.unlock();}
  WaveOut readwaveout(){return wout;}
  
  void setsweepcount(unsigned int msweepcount){mlock.lock();sweepcount=msweepcount;mlock.unlock();};
  unsigned int readsweepcount(){return sweepcount;}

  void setmode(WorkMode mmode){mode=mmode;}
  WorkMode readmode(){return mode;}

  void setstartfreq(double mstartfreq){mlock.lock();startfreq=mstartfreq;mlock.unlock();}
  double readstartfreq(){return startfreq;}
  
  void setstopfreq(double mstopfreq){mlock.lock();stopfreq=mstopfreq;mlock.unlock();}
  double readstopfreq(){return stopfreq;}

  void setstepfreq(double mstepfreq){mlock.lock();stepfreq=mstepfreq;mlock.unlock();}
  double readstepfreq(){return stepfreq;}

  void setdirection(SweepDirection mdirection){mlock.lock();direction=mdirection;mlock.unlock();}
  SweepDirection readdirection(){return direction;}
  
  void setrunstop(RunStop mrunstop){mlock.lock();run_stop=mrunstop;mlock.unlock();}
  RunStop readrunstop(){return run_stop;}

  void setdwell(double mdwell){mlock.lock();dwell=mdwell;mlock.unlock();}
  double readdwell(){return dwell;}

  void setdetr(Detector mdetr){mlock.lock();detr=mdetr;mlock.unlock();}
  Detector readdetr(){return detr;}

  void setdemod(Demodulation mdemod){mlock.lock();demod=mdemod;mlock.unlock();}
  Demodulation readdemod(){return demod;}
  

  void setgpsopen(bool mopen){mlock.lock();gpsopen=mopen;mlock.unlock();}
  bool readgpsopen(){return gpsopen;}

  void settunedelay(int mdelay){mlock.lock();tune_delay=mdelay;mlock.unlock();}
  int gettunedelay(){return tune_delay;}
  
  void setdwelldelay(int mdelay){mlock.lock();dwell_delay=mdelay;mlock.unlock();}
  int getdwelldelay(){return dwell_delay;}
  
  void setDPram(DigPram dpram){mlock.lock(); dPram=dpram; mlock.unlock();}
  DigPram getDPram(){return dPram;}
  
  void setGain(int wgain){mlock.lock(); gain=wgain; mlock.unlock();}
  int getGain(){return gain;}
  
  void setWin(int ws){mlock.lock();winstyle=ws;mlock.unlock();}
  int getWin(){return winstyle;}
  
  
  void showmessage();
  
  void settimezone(float mtime){mlock.lock(); timezone=mtime;mlock.unlock();}
  
  float  gettimezone(){return timezone;}
  
  void  set_device_name(std::string mstr, std::string dstr){
	  mboard_name=mstr;
	  dboard_name=dstr;
  }
       
};


#define SBSIZE 8192
class SendBuf
{
  Mutex mlock;

  short  voicebuf[SBSIZE];
  int    fbuf[SBSIZE];
  int    dbbuf[SBSIZE];

  int    FFM[12]; 
  int    bufindex;
  bool   sendbegin;
  bool   FFMok;
  int    packetlength;
  int    packetnum;
  int    kinds;
  double startfreq;
public:

  void write(int *infbuf, int *indbbuf,int len);
  void read(int *outfbuf,int *outdbbuf,int len);
  void writev(short *inbuf, int len);
  void readv(short *outbuf,int len);
  int readbufindex();
  void showbuf();
  void clear();
  bool readsendstate(){return sendbegin;}
  void setsendstate(bool mstate){
              mlock.lock();
              sendbegin=mstate; 
              mlock.unlock();
        }
  void setpacket(int len,int num,double sfreq){
              packetlength=len;
              packetnum=num;
              startfreq=sfreq;

        }
  void setkinds(int mkinds){
               mlock.lock();
               kinds=mkinds;
               mlock.unlock();
        }
  void  setFFM(int p1,int p2,int p3,int p4,int *timebuf){
          mlock.lock();
          FFM[0]=p1;
          FFM[1]=p2;
          FFM[2]=p3;
          FFM[3]=p4;
          memcpy(FFM+4,timebuf,8*sizeof(int));
          //cout<<"FFM =";
         // for(int i=0;i<13;i++) cout<<FFM[i]<<" ";
          //cout<<endl;
          mlock.unlock();
        }
  void  setFFM(bool ffmok){mlock.lock();FFMok=ffmok;mlock.unlock();}
  void  readFFM(int *buf){
           memcpy(buf,FFM,12*sizeof(int));
          // cout<<"buf=";
          // for(int i=0;i<13;i++) cout<<buf[i]<<" ";
           //cout<<endl;
        }

  bool readFFMstate(){return FFMok;}
  int  readkinds(){return kinds;}
  int  packet_len(){return packetlength;}
  int  packet_num(){return packetnum;}
  double start_freq(){return startfreq;}
  SendBuf();
  ~SendBuf(){};
};


#endif


