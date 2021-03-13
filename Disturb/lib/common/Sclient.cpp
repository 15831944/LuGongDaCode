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

#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <common/Sclient.h>
#include <common/Types.hpp>
#include <common/Configuration.h>


using namespace std;    
 
void show(uint32_t x)
{
   uint32_t x1,x2,x3,x4;
   x1=x & 0xff000000;
   x2=x & 0x00ff0000;
   x3=x & 0x0000ff00;
   x4=x & 0x000000ff;
   x1=x1>>24;
   x2=x2>>16;  
   x3=x3>>8;
   cout<<x4<<"."<<x3<<"."<<x2<<"."<<x1;
}

Client::Client(COMMessage *lmessage, SendBuf *lfifo)
{
  fifo=lfifo;
  message=lmessage;
  CPORT=6003;
  bzero(&server_addr,sizeof(server_addr));
  server_addr.sin_family=AF_INET;
  server_addr.sin_addr.s_addr = htons(INADDR_ANY);
  server_addr.sin_port=htons(CPORT);

  busy=false;
  bzero(&server_udp,sizeof(server_udp));
  server_udp.sin_family=AF_INET;
  server_udp.sin_port=htons(CPORT);


  sockt_tcp=socket(AF_INET,SOCK_STREAM,0);//tcp socket
  sockt_udp=socket(AF_INET,SOCK_DGRAM,0);// udp socket


//============fixed me========================
//wait_time is very important, but i set it as zero for convernience
//it should be set to a proper value. 
  struct linger linger;
  linger.l_onoff=1;
  linger.l_linger=0;
  setsockopt(sockt_tcp,SOL_SOCKET,SO_LINGER,(const char*)&linger,sizeof(linger));  
  if( -1==bind(sockt_tcp,(struct sockaddr*)&server_addr,sizeof(server_addr)))
  {
      printf("Server Bind Port : %d Failed!\n", CPORT);
      exit(1);
  }

  if(-1==listen(sockt_tcp,5)){
       printf("listen error\n");
  }

};
Client::~Client()
{
   close(sockt_tcp);
   close(sockt_udp);
   close(sockt_in);
}

void Client::setserverip(char *ipstr)
{
   if(inet_pton(AF_INET,ipstr,&server_udp.sin_addr)<=0){
      printf("[%s] is not a valid IP address\n)",ipstr);
      exit(1);
    }
}

bool Client::connectserver()
{
   //char ipstr[]="26.28.211.62";
   //setserverip(ipstr);
   // show(server_udp.sin_addr.s_addr);
   if(connect(sockt_udp,(struct sockaddr*)&server_udp,sizeof(server_udp))>=0)
     return true;
   else{
     perror("connect to targus soft error"); 
     return false;
   }

}
void Client::start()
{
   
   sendthread.start((void* (*)(void*))sendpro, (void*)this);
   recievethread.start((void* (*)(void*))recievepro, (void*)this);
   onconnectthread.start((void* (*)(void*))onconnectpro, (void*)this);
}

void Client::send()
{
  //tcp 
   // there may be something wrong, beacause we don't check wethear the sockt_in is ok.
	if(fifo->readsendstate())
	{
		int sendbuf[5];
		sendbuf[0]=1; //means send begin message sign;
		sendbuf[1]=fifo->packet_len();
		sendbuf[2]=fifo->packet_num();
		cout<<"  TCP sending"<<endl;
		double startfreq=fifo->start_freq();
		sendbuf[3]=floor(startfreq/2147483648);
		sendbuf[4]=startfreq - (double)sendbuf[3]*2147483648;
		write(sockt_in,sendbuf,5*sizeof(int));
		fifo->setsendstate(false);
	}
  //udp data of frequency level stars
	int N=fifo->packet_len();
	if (fifo->readbufindex()>=N)  
	{
		int kinds=fifo->readkinds();// the style of packets  voice data or frequency-level data
		//cout<<"kinds="<<kinds<<" packet_len="<<N<<endl;
		switch(kinds){
			case 0://kinds=0 frequency-level data
			case 3:{//kinds=3 stars
				int z[2*N+1];
				z[0]=kinds; 
				fifo->read(z+1,z+N+1,N);
				socklen_t len=sizeof(server_udp);
				sendto(sockt_udp,z,(2*N+1)*sizeof(int),0,(struct sockaddr*) &server_udp,len);
				}
				break;
			case 1://kinds=1 voice data;
			case 4:{//kinds=1 bits data
				short z[N+2];
				z[0]=kinds;  
				z[1]=0;
				fifo->readv(z+2,N);
				socklen_t len=sizeof(server_udp);
				sendto(sockt_udp,z,(N+2)*sizeof(short),0,(struct sockaddr*) &server_udp,len);
				}
				break;
          
		}
	}
//  FFM mode 因为数据量太少，所以单独成为一种模式
	if (fifo->readFFMstate())
	{
		int FFM_packet[12+1];  
		int kinds=2;//kinds=2; ffm data;
		FFM_packet[0]=kinds;
		fifo->readFFM(FFM_packet+1);
		socklen_t len=sizeof(server_udp);
		sendto(sockt_udp,FFM_packet,(12+1)*sizeof(int),0,(struct sockaddr*) &server_udp,len);
		fifo->setFFM(false); 
	}
}

void Client::voicesend(short *buf,int size)
{
  sendto(sockt_in,buf,size*2,0,(struct sockaddr*) &server_udp,sizeof(server_udp));
}

void Client::gpssend(int *buf,int len)
{
   int sendbuf[32];
   sendbuf[0]=3; 
   memcpy(sendbuf+1,buf,len*sizeof(int));
   write(sockt_in,sendbuf,(len+1)*sizeof(int));
}

bool Client::readconnectstate()
{
   return busy;
}

void Client::sendparameter()
{


}

void Client::recieve()
{
    char recvline[1024];
    socklen_t len=sizeof(pcliaddr);
   // int n=read(sockt_new,recvline,1024);
    int n=recvfrom(sockt_in,recvline,1024,0,(struct sockaddr*) &pcliaddr,&len);
    if (busy and (n==0)) {cout<<"  disconnect"<<endl;message->setrunstop(STOP);message->setgpsopen(false);busy=false;}
    if(n>0){

      recvline[n]='\0';
      //cout<<recvline<<endl;
      //write(sockt_tcp,recvline,n+1);
      BackType back;
      prase_init(recvline,message,&back);
      //任何参数的下达都是采用 ASCII 体制，暂时还没有改成二进制格式的必要
      //back 是一个回传参数，目前的设计是作为回应windows端指令的一个标示。
      response(back);
    }
}
void Client::send_device_serial()
{
	int sendbuf[256];
	int cid=8;
	std::string mboard_serial=message->mboard_name;
	std::string dboard_serial=message->dboard_name;
	int len1=strlen((char*) mboard_serial.c_str());
	int len2=strlen((char*) dboard_serial.c_str());
	
	//cout<<mboard_serial<<" "<<dboard_serial<<endl;
	void **wp=new void *;
	*wp=sendbuf;
	writebuf(wp,&cid,4);
	writebuf(wp,&len1,4);
	writebuf(wp,&len2,4);
	writebuf(wp,(char*) mboard_serial.c_str(),len1);
	writebuf(wp,(char*) dboard_serial.c_str(),len2);
	int sendlen=(floor((len1+len2)/4)+4)*4;
	write(sockt_in,sendbuf,sendlen);
	
}
void Client::adjfilesend(int *buf,int len)
{
   int sendbuf[256];
   sendbuf[0]=9; 
   memcpy(sendbuf+1,buf,len*sizeof(int));
   write(sockt_in,sendbuf,(len+1)*sizeof(int));
}
void Client::cope_ask_adj()
{
	send_device_serial();
	usleep(200000);
	FILE *fp;
	fp=fopen("db_value_adjust.adj","r");
	int adnumber=0;
	double adfreq[1024],addb[1024];
	if(fp>0){
		cout<<endl;
		cout<<endl;
		int readnumber;
		fscanf(fp,"%d\n",&readnumber);
		fscanf(fp,"%lf,%lf\n",&adfreq[0],&addb[0]);
		cout<<"  freq="<<adfreq[0]<<" db="<<addb[0]<<endl;
		adnumber=adnumber+1;
		for(int i=1;i<readnumber;i++){
			fscanf(fp,"%lf,%lf\n",&adfreq[i],&addb[i]);
			if(adfreq[i]<=adfreq[i-1]) {
				cout<<"  Warning adjust file error at line "<<i+2<<endl;
				break;
			}
			adnumber=adnumber+1;
			cout<<"  freq="<<adfreq[i]<<" db="<<addb[i]<<endl;
		}
		fclose(fp);
		
		cout<<endl<<"  adnumber="<<adnumber<<endl<<endl;
	}
	else{
		cout<<"  warning adjust file doesn't exist"<<endl;
	}
	int tempbuf[240];//about 240*4=960bytes
	int index=1;
	tempbuf[0]=-1;//文件开始发送标志
	tempbuf[1]=460;//母板标志
	tempbuf[2]=1; //子板标志
	int needsend=adnumber+1;
	int packetnum=240;
	int j=0;
	while(needsend>0){
		if (needsend>packetnum){
			for(int i=index;i<packetnum;i++){
				tempbuf[3*i]=floor(adfreq[j]/2147483648);
				tempbuf[3*i+1]=adfreq[j]-double(tempbuf[3*i])*2147483648;
				tempbuf[3*i+2]=addb[j]*1000;
				j=j+1;
			}
			adjfilesend(tempbuf,packetnum*3);
			needsend=needsend-packetnum;
			index=0;
		}else{
			for(int i=index;i<needsend;i++){
				tempbuf[3*i]=floor(adfreq[j]/2147483648);
				tempbuf[3*i+1]=adfreq[j]-double(tempbuf[3*i])*2147483648;
				tempbuf[3*i+2]=addb[j] *1000;
				j=j+1;
			}
			adjfilesend(tempbuf,needsend*3);
			needsend=needsend-needsend;
			index=0;
			
		}
		usleep(200000);
		
	}
	usleep(200000);
	cout<<"  send tail"<<endl;
	tempbuf[0]=-1;
	tempbuf[1]=-1;
	tempbuf[2]=0;
	adjfilesend(tempbuf,3);
	
}
void Client::response(BackType mback)
{
	switch(mback){
		case ASKING_ADJF:cope_ask_adj();cout<<"  asking adjust file"<<endl;break;
		default: break;
	}
}
void Client::onconnect()
{
  socklen_t len=sizeof(pcliaddr);
  int socket_new;

  if((socket_new= accept (sockt_tcp,    (struct sockaddr*) &pcliaddr, &len))>0){



     int ok=1;
     int no=0;
     int wrong=2;
     int sendbuf[2];
     sendbuf[0]=0; //means connect state message sign
     if (not busy){
        server_udp.sin_addr.s_addr=pcliaddr.sin_addr.s_addr;
        if (connectserver()){
         sendbuf[1]=ok;
         write(socket_new,sendbuf,2*sizeof(int));
         
         
         busy=true;
         sockt_in=socket_new;
         usleep(200000);
         send_device_serial();
         cout<<"user ";
         show(pcliaddr.sin_addr.s_addr);
         cout<<" is connected"<<endl;
        }
        else{
         sendbuf[1]=wrong;
         write(socket_new,sendbuf,2*sizeof(int));
        }
        
     }
     else{
       sendbuf[1]=no;
       write(socket_new,sendbuf,2*sizeof(int));
       //close(socket_new);
     }
     //cout<<"accept"<<endl;    
  }  
}

void *sendpro(Client* client)
{
  while (1) {
    client->send();
    pthread_testcancel();
    usleep(0);
  }
  return NULL;
}

void *recievepro(Client* client)
{
  while (1) {
    client->recieve();
    pthread_testcancel();
    usleep(0);
  }
  return NULL;
}

void *onconnectpro(Client *client)
{
   while(1){
     client->onconnect();
     pthread_testcancel();
     usleep(0);
   }
}
