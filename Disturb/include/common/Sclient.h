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
#ifndef SCLIENT_H
#define SCLIENT_H

#include <netinet/in.h>    // for sockaddr_in
#include <sys/types.h>    // for socket
#include <sys/socket.h>    // for socket
#include <stdio.h>        // for printf
#include <stdlib.h>        // for exit
#include <string.h>        // for bzero

#include <arpa/inet.h>
#include <unistd.h>
#include <math.h>

#include <common/Thread.h>
#include <common/SCPIPrase.h>
 



class Client
{
protected:
  Thread          sendthread;
  Thread          recievethread;
  Thread          onconnectthread;
  SendBuf         *fifo;
  COMMessage      *message; // a model which restore the command message
  int			  CPORT;
  int             sockt_tcp;
  int             sockt_udp;  
  int             sockt_in;
  struct          sockaddr_in server_addr,pcliaddr,server_udp;
  bool            busy;
public:
  Client(COMMessage *lmessage, SendBuf *lfifo );
  ~Client();
  void setserverip(char *ipstr);
  void start();
  void send();
  void voicesend(short *buf, int size);
  void gpssend(int *buf,int len);
  void adjfilesend(int *buf,int len);
  void send_device_serial();
  void cope_ask_adj();
  void recieve();
  void onconnect();
  void response(BackType rback);
  bool connectserver();
  void sendparameter();
  bool readconnectstate();
  friend void *onconnectpro(Client *);
  friend void *sendpro(Client*);
  friend void *recievepro(Client*);


};
void *sendpro(Client*);
void *recievepro(Client*);
void *onconnectpro(Client *);

#endif

