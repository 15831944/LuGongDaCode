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

#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <common/SCPIPrase.h>
using namespace std;


void settimebuf(int stimebuf[])
{
	struct tm _tm;
	time_t timep;
	timeval tv;
	_tm.tm_sec=stimebuf[5];
	_tm.tm_min=stimebuf[4];
	_tm.tm_hour=stimebuf[3];  
	_tm.tm_mday=stimebuf[2];
	_tm.tm_mon=stimebuf[1]-1;
	_tm.tm_year=stimebuf[0]-1900;

	timep= mktime(&_tm);
	
	tv.tv_sec=timep;
	tv.tv_usec=0;
	if(settimeofday(&tv, (struct timezone *)0)<0){
		cout<<"time set error"<<endl;	
	}

}
   
//it is not wiseable to attach pionter to realsed memory
// so if memory is allocated in son function, the return pointer attached to this memory may create error.
void loadwordsfromstr(char* buf,char *words) 
{
  bool isendstr=true;
  bool isendwords=false;
  int i=0;
  if (buf[i]=='\0') return;

  //cout<<"input buf is: "<<buf<<endl;

  while(buf[i]!='\0'){
    isendwords=(buf[i]==' ');
    isendwords=(isendwords or (buf[i]==':'));
    isendwords=(isendwords or (buf[i]==';'));
    if(not isendwords){
      assert(i<MWL);
      words[i]=buf[i]; 
    }
    else{
      isendstr=false;
      break;
    }
    i=i+1; 
  }

  words[i]='\0';
  int k=0; 
  if(not isendstr){
    while(buf[k+i+1]!='\0'){
       buf[k]=buf[k+i+1];
       k=k+1;
    }
    buf[k]='\0';
 //   cout<<"new buf is: "<<buf<<endl;
  }
  else
    buf[0]='\0';
//  cout<<"words is: "<<words<<endl;
  /* cout<<"m="<<k+1<<endl;
   cout<<"length="<<strlen(buf)<<endl;
   for(int m=k+1;m<MWL;m++) buf[m]=0;
   cout<<"length="<<strlen(buf)<<endl;
   cout<<"after buf="<<buf<<endl;*/
  return;
}

void loadstrfromtext(char* buf, char *str) 
{
  bool isendtext=true;
  bool isendstr=false;
  int i=0;
  if (buf[i]=='\0') return;

  //cout<<"input buf is: "<<buf<<endl;

  while(buf[i]!='\0'){
    isendstr=(buf[i]==';');
    isendstr=(isendstr or (buf[i]=='\n'));
    if(not isendstr){
      str[i]=buf[i];
    }
    else{
      isendtext=false;
      break;
    }
    i=i+1; 
  }
  str[i]='\0';
  if(not isendtext){
    int k=0; 
    while(buf[k+i+1]!='\0'){
       buf[k]=buf[k+i+1];
       k=k+1;
    }
    buf[k]='\0';
    //cout<<"new buf is: "<<buf<<endl;
  }
  else
    buf[0]='\0';
  //cout<<"str is: "<<str<<endl;
  return;
}

void prase_init(char* buf, COMMessage *message,BackType *back)
{
	
 //cout<<(int)buf[0]<<endl;
	switch (buf[0]){
		case -1://Dpram;
		{
			cout<<"recive dPram"<<endl;
			DigPram dpram;
			BYTE header;
			void **wp= new void *;//必须先要为二重指针分配一块内存
			*wp=buf;
			readbuf(wp,&header,1);
			readbuf(wp,&dpram,sizeof(dpram));
			showDigPram(dpram);
			message->setDPram(dpram);
			return;
		}
	}
	*back=NONE;
	char comstr[MSL];
	while(buf[0]!='\0'){
		loadstrfromtext(buf,comstr);
		cout<<"  "<<comstr<<endl;
		prase_com(comstr,message,back);
		cout << "\n";
	}  
}




void prase_com(char* buf, COMMessage *message,BackType *back)
{
  char comword[MWL];

  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
  //  cout<<"comword:"<<comword<<endl;
  //  cout<<"buf="<<buf<<endl;
  //  cout<<"buf="<<buf<<endl;
   // if(comword[0]!=0)
      cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"SENSe")==0){ prase_sense(buf,message,back); return;}
    if (strcmp(comword,"INITiate")==0){ prase_initiate(buf,message,back); return;}
    if (strcmp(comword,"ABORT")==0){  message->setrunstop(STOP); return;}
    if (strcmp(comword,"TIME")==0){
      int pnum=0;
      int timebuf[6];
      while(buf[0]!='\0'){  

        char timepra[MWL];
        loadwordsfromstr(buf,timepra);
        timebuf[pnum]=prase_parameter(timepra);
        pnum=pnum+1;
      }
      if (pnum==6) settimebuf(timebuf);
      return;
    }

  }
  if(comword[0]!=0)
    cout<<"  There is no "<<"\""<<comword<<"\""<<"in root level command"<<endl;
  return;
}



void prase_initiate(char* buf, COMMessage *message, BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    if (strcmp(comword,"IMMediate")==0){  message->setrunstop(RUN); message->showmessage();return;}
    if (strcmp(comword,"CONM")==0){
      if(buf[0]!='\0') loadwordsfromstr(buf,comword); 
      if (strcmp(comword,"IMMediate")==0)
      message->setrunstop(CONTINUE);
      return;
    }
  }
 
}



void prase_sense(char* buf, COMMessage *message, BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"FREQuency")==0){ prase_sense_freq(buf,message,back); return;}
    if (strcmp(comword,"SWEep")==0){ prase_sense_sweep(buf,message,back); return;}
    if (strcmp(comword,"DETector")==0){ prase_sense_detector(buf,message,back); return;}
    if (strcmp(comword,"WAVeout")==0){ prase_sense_waveout(buf,message,back); return;}
    if (strcmp(comword,"DEModulation")==0){ prase_sense_demodulation(buf,message,back); return;}
    if (strcmp(comword,"GPS")==0){ prase_sense_gps(buf,message,back); return;}
    if (strcmp(comword,"TUNEdelay")==0){ prase_sense_tune(buf,message,back); return;}
    if (strcmp(comword,"DWELldelay")==0){ prase_sense_dwell(buf,message,back); return;}
    if (strcmp(comword,"TIMEzone")==0){ prase_sense_timezone(buf,message,back); return;}
    if (strcmp(comword,"GAIN")==0){ prase_sense_gain(buf,message,back); return;}
    if (strcmp(comword,"FFTWin")==0) { prase_sense_win(buf,message,back);return;}
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in SENSe command"<<endl; 
}

void prase_sense_gain(char* buf,COMMessage *message,BackType *back)
{
  if(buf[0]!='\0'){
    int gain=prase_parameter(buf);
    message->setGain(gain);
    cout<<"  Gain is set to be "<<gain<<" dB"<<endl;
  }	
}

void prase_sense_win(char* buf,COMMessage *message,BackType *back)
{
  if(buf[0]!='\0'){
    int ws=prase_parameter(buf);
    message->setWin(ws);
    cout<<"  WinSytle is set to be "<<ws<<endl;
  }	
}


void prase_sense_tune(char* buf,COMMessage *message,BackType *back)
{
  if(buf[0]!='\0'){
    int tune_delay=prase_parameter(buf);
    message->settunedelay(tune_delay);
    cout<<"  Tune Delay is set to be "<<tune_delay<<" ms"<<endl;
  }	
}

void prase_sense_dwell(char* buf,COMMessage *message,BackType *back)
{
  if(buf[0]!='\0'){
    int dwell_delay=prase_parameter(buf);
    message->setdwelldelay(dwell_delay);
    cout<<"  Dwell Delay is set to be "<<dwell_delay<<" ms"<<endl;
  }	
}

void prase_sense_timezone(char* buf,COMMessage *message,BackType *back)
{
  if(buf[0]!='\0'){
    float timezone=prase_parameter(buf);
    message->settimezone(timezone);
    cout<<"  Time zone is set to be "<<timezone<<" hours"<<endl;
  }	
}

void prase_sense_gps(char* buf,COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"ON")==0){message->setgpsopen(true); return;}
    if (strcmp(comword,"OFF")==0){message->setgpsopen(false);  return;}
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in GPS mode"<<endl; 
}


void prase_sense_demodulation(char* buf,COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"AM")==0){message->setdemod(AM); return;}
    if (strcmp(comword,"FM")==0){message->setdemod(FM);  return;}
    if (strcmp(comword,"OFF")==0){message->setdemod(OFF);  return;}
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in demodulaton mode"<<endl; 
}

void prase_sense_waveout(char* buf,COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"OFF")==0){message->setwaveout(WOFF); return;}
    if (strcmp(comword,"SOFT")==0){message->setwaveout(SOFT);  return;}
    if (strcmp(comword,"HARD")==0){message->setwaveout(HARD);  return;}
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in demodulaton mode"<<endl; 
}

void prase_sense_detector(char* buf,COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"PAVerage")==0){message->setdetr(PAV); return;}
    if (strcmp(comword,"POSitive")==0){message->setdetr(PMAX);  return;}
    if (strcmp(comword,"FAST")==0){ message->setdetr(FAST);return;}
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in detector mode"<<endl; 
}

void prase_sense_freq(char* buf, COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"STARt")==0){
      if(buf[0]!='\0'){
        double freq=prase_parameter(buf);
        message->setstartfreq(freq);
        cout<<"  Start frequency is set to be "<<freq<<"Hz"<<endl;
      }
      return;
    }
    if (strcmp(comword,"STOP")==0){
      if(buf[0]!='\0'){
        double freq=prase_parameter(buf);
        message->setstopfreq(freq);
        cout<<"  Stop frequency is set to be "<<freq<<"Hz"<<endl;
      }
      return;
    }
    if (strcmp(comword,"STEP")==0){
      if(buf[0]!='\0'){
        double freq=prase_parameter(buf);
        message->setstepfreq(freq);
        cout<<"  Step frequency is set to be "<<freq<<"Hz"<<endl;
      }
      return;  
    }
    if (strcmp(comword,"LIST")==0){
      
      while(buf[0]!='\0'){  
        //cout<<"buf="<<buf<<endl;  
           
        char listfreq[MWL];
        loadwordsfromstr(buf,listfreq);
	 if (strcmp(listfreq,"STARt")==0)
	 {
	   message->fnum=0;
	   cout<<"  freq list begin"<<endl;
	 }
	 else
	 {
        //cout<<"listfreq="<<listfreq;   
          double freq=prase_parameter(listfreq);
          message->flist[message->fnum]=freq;
          message->fnum=message->fnum+1;
          cout<<"  add "<<freq<<"Hz"<<" to list"<<endl;
	}  
      }
      return;
    }
    if (strcmp(comword,"ADJUst")==0){
      
      while(buf[0]!='\0'){  
        //cout<<"buf="<<buf<<endl;  
           
        char listfreq[MWL];
        char listdb[MWL];
        loadwordsfromstr(buf,listfreq);
        if(strcmp(listfreq,"GET")==0){
			*back=ASKING_ADJF;
			return;
		}
        loadwordsfromstr(buf,listdb);
		if (strcmp(listfreq,"STARt")==0)
		{
			//message->fnum=0;
			int adnum=str2int(listdb);//文件开始时的listdb 表示校准表的长度
			FILE *fpadj;
			fpadj=fopen("db_value_adjust.adj","w");  //建立新文件，相当于重新更新
			if(fpadj>0){
				fprintf(fpadj,"%d\n",adnum);
				fclose(fpadj);
			}else cout<<" WARNNING can't creat adjust file"<<endl;
			
			cout<<" adjust list transfer begin num="<<adnum<<endl;
		}
		else
		{
			//cout<<"listfreq="<<listfreq;   
			double freq=prase_parameter(listfreq);
			double dbvalue=prase_parameter(listdb);
			
			
			FILE *fpadj;
			fpadj=fopen("db_value_adjust.adj","a");//追加的方式添加
			if(fpadj>0){
				fprintf(fpadj,"%f,%f\n",freq,dbvalue);
				fclose(fpadj);
			}else cout<<" WARNNING can't open adjust file"<<endl;

			cout<<"  add freq="<<freq<<" db="<<dbvalue<<" to list"<<endl;
		}  
      }
      return;
    }
    if (strcmp(comword,"MODE")==0){
      if(buf[0]!='\0'){
        char modestr[MWL];
        loadwordsfromstr(buf,modestr);
        WorkMode mode=NOMODE;
        if (strcmp(modestr,"SWEep")==0) mode=FSCAN;
        if (strcmp(modestr,"DSCan")==0) mode=DSCAN;
        if (strcmp(modestr,"CW")==0) mode=FFM;
        if (strcmp(modestr,"MSCan")==0) mode=FLS;
        if (strcmp(modestr,"DIGM")==0) mode=DIGM;
        if (strcmp(modestr,"DIGS")==0) mode=DIGS;
        if (mode!=NOMODE){
          message->setmode(mode);
          cout<<"  Frequency mode is set as "<<modestr<<endl;
	}
        else
          cout<<"  "<<modestr<<" is not a valid work mode"<<endl;          
      }
      return;
    }
  }
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in SENSe command"<<endl; 
}

void prase_sense_sweep(char* buf, COMMessage *message,BackType *back)
{
  char comword[MWL];
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,comword);
    cout<<"  ["<<comword<<"]"<<endl;
    if (strcmp(comword,"STEP")==0){
      if(buf[0]!='\0'){
        double freq=prase_parameter(buf);
        message->setstepfreq(freq);
        cout<<"  Step frequency is set to be "<<freq<<"Hz"<<endl;
      }
      return;
    }
    if (strcmp(comword,"COUNt")==0){
      if(buf[0]!='\0'){
        unsigned int  count=prase_parameter(buf);
        message->setsweepcount(count);
        if (count<=65535) 
          cout<<"  Sweep count is set to be "<<count<<" times"<<endl;
        else 
          cout<<"  Sweep count is set to be "<<"INF"<<" times"<<endl;
      }
      return;
    }

    if (strcmp(comword,"DWELl")==0){
      if(buf[0]!='\0'){
        double dwell=prase_parameter(buf);
        message->setdwell(dwell);
        cout<<"  Sweep dwell is set to be "<<dwell<<"s"<<endl;
      }
      return;
    }

    if (strcmp(comword,"DIRection")==0){
      if(buf[0]!='\0'){
        char directionstr[MWL];
        loadwordsfromstr(buf,directionstr);
        SweepDirection direction=NODIRECTION;
        if (strcmp(directionstr,"UP")==0) direction=UP;
        if (strcmp(directionstr,"DOWN")==0) direction=DOWN;
        if (direction!=NODIRECTION){
          message->setdirection(direction);
          cout<<"  Sweep direction is set as "<<directionstr<<endl;
	}
        else
          cout<<"  "<<directionstr<<" is not a valid sweep direction"<<endl;  
      }
      return;
    }
   }  
  cout<<"  There is no "<<"\""<<comword<<"\""<<"in Sweep command"<<endl;   
}






double prase_parameter(char* buf)
{
  char valuestr[MWL];
  char unitstr[MWL];
  double unit=1;
  loadwordsfromstr(buf,valuestr);
  double value=str2double(valuestr);
  if(buf[0]!='\0'){
    loadwordsfromstr(buf,unitstr);
    switch(unitstr[0]){
      case 'K':unit=1e3;break;
      case 'M':unit=1e6;break;
      case 'G':unit=1e9;break;
      default: unit=1;break; 
    }
  }
  
  return value*unit; 
  
}


double str2double(char *buf)
{

  int i=0;
  double value=0;
  char basestr[32];
  int  pointpos=0;
  char powerstr[32];
  int stage=1;
  int m=0;
  int n=0;
  while(buf[i]!='\0'){
    switch(stage){
    case 1:
      if (buf[i]=='.'){
        stage=2;
        break;
      }
      if (buf[i]=='e'){
        stage=3;
        break;
      }
      basestr[m]=buf[i];
      m=m+1;
      break;
    case 2:
      if (buf[i]=='e'){
        stage=3;
        break;
      }
      basestr[m]=buf[i];
      m=m+1;
      pointpos=pointpos+1;
      break;
    case 3:
      powerstr[n]=buf[i];
      n=n+1;
      break;
    }
    i=i+1;
  }
  basestr[m]='\0';
  powerstr[n]='\0';
  //cout<<basestr<<endl;
  //cout<<powerstr<<endl;
  //cout<<pointpos<<endl;
  value=str2int(basestr)*pow(10,(str2int(powerstr)-pointpos));
  //cout<<value<<endl;
  return value;
}
long long int str2int(char *buf)
{
  int i=0;
  long long int value=0;
  int sign=1;
  if (buf[0]=='-'){sign=-1;i=i+1;} 
  while(buf[i]!='\0'){
    if ((buf[i]>57) or (buf[i]<48)) return 0;
    value=(value*10)+(buf[i]-48);
    i=i+1;
  }
  return value*sign;  
}


