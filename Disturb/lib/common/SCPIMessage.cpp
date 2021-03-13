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
/*define demo to avoid the acquire of usrp device*/
//#define  demo



#include <iostream>    

#include <common/SCPIMessage.h>
using namespace std;


void COMMessage::resest()
{
  sweepcount=1;//sweep frequency times
  mode=DSCAN;  //working mode
  startfreq=2002e6; //start frequency
  stopfreq=947e6;
  stepfreq=3.125e3;
  
  direction=DOWN;  //sweep direction
  detr=PAV;
  demod=FM;
  run_stop=STOP;  //state of SpectrumAnalyzeDevice
  dwell=0.005;
} 

void COMMessage::showmessage()
{
  cout<<"==============Parameter of SDSA ==============="<<endl;
  if (sweepcount<=65535)
    cout<<"  sweepcount:     "<<sweepcount<<endl;
  else
    cout<<"  sweepcount:     "<<"INF"<<endl;
  cout<<"  workmode:       "<<workmode[mode]<<endl;
  cout<<"  startfreq:      "<<startfreq<<endl;
  cout<<"  stopfreq:       "<<stopfreq<<endl;
  cout<<"  stepfreq:       "<<stepfreq<<endl;
  cout<<"  gain:           "<<gain<<endl;
  cout<<"  dwelltime:      "<<dwell<<endl;
  cout<<"  sweepdirection: "<<sdirection[direction]<<endl;
  cout<<"  detector:       "<<detector[detr]<<endl;
  cout<<"  demodulation:   "<<demodulation[demod]<<endl;
  cout<<"  run_stop:       "<<runstop[run_stop]<<endl;     

}





SendBuf::SendBuf()
{
  bufindex=0; 
  sendbegin=false;
  FFMok=false;
  packetlength=256;
}
void SendBuf::write(int *infbuf, int *indbbuf,int len)
{
   mlock.lock();
   int datasize=sizeof(int);
  // assert((bufindex+len)<SBSIZE);// the sign of overflow
   if((bufindex+len)<SBSIZE){
   memcpy(fbuf+bufindex,infbuf,len*datasize);
   memcpy(dbbuf+bufindex,indbbuf,len*datasize);
   bufindex=bufindex+len;
  }else cout<<"   warning fifo is full"<<endl;
   mlock.unlock();  
   
}
void SendBuf::writev(short *inbuf,int len)
{
   mlock.lock();
   int datasize=sizeof(short);
   //assert((bufindex+len)<SBSIZE);// the sign of overflow
   if((bufindex+len)<SBSIZE){
   memcpy(voicebuf+bufindex,inbuf,len*datasize);
   bufindex=bufindex+len;
   } else cout<<"   warning voice buf fifo is full"<<endl;
  
   mlock.unlock();
   
}





void SendBuf::read(int *outfbuf,int *outdbbuf,int len)
{
   mlock.lock();
   int datasize=sizeof(int);
   assert((bufindex-len)>=0);// the sign no data to read
   memcpy(outfbuf,fbuf,len*datasize);
   memcpy(outdbbuf,dbbuf,len*datasize);
   if (bufindex-len>0)
   {
      memcpy(fbuf,fbuf+len,(bufindex-len)*datasize);
      memcpy(dbbuf,dbbuf+len,(bufindex-len)*datasize);
   }
   bufindex=bufindex-len;
   //showbuf();
   mlock.unlock();
}

void SendBuf::readv(short *outbuf,int len)
{
   mlock.lock();
   int datasize=sizeof(short);
   assert((bufindex-len)>=0);// the sign no data to read
   memcpy(outbuf,voicebuf,len*datasize);
   if (bufindex-len>0)
   {
      memcpy(voicebuf,voicebuf+len,(bufindex-len)*datasize);
   }
   bufindex=bufindex-len;
   //showbuf();
   mlock.unlock();
}
     

int SendBuf::readbufindex()
{ 
  return bufindex;
}


  
void SendBuf::showbuf()
{
  for(int i=0;i<bufindex;i++)
     cout<<fbuf[i]<<"/"<<dbbuf[i]<<" ";
  cout<<endl;
}

void SendBuf::clear()
{
  bufindex=0;
  sendbegin=false;
  packetlength=256;
  FFMok=false;
}
