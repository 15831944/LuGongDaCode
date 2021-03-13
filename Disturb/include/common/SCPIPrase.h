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
#ifndef PRASE_COM_H
#define PRASE_COM_H
#include <string.h>
#include <math.h>
#include <common/SCPIMessage.h>
#include <common/Types.hpp>



//MSL the maximum length of command;
//MWL the maximum length of word;
#define MSL 2048
#define MWL 128

//回传动作标示：解析命令后，如果要做出反应，则需要将该值回传
//主要应用与windows 和 linux 之间命令和参数的交互
enum BackType {NONE,
				ASKING_ADJF};  
/*
 *NONE 不用给出任何回应
 *ADJF 要求回传 adjust db file 
 *   
 */
void loadwordsfromstr(char* buf,char *words);
void loadstrfromtext(char* buf, char *str); 
void prase_init(char* buf, COMMessage *message,BackType *back);
void prase_com(char* buf, COMMessage *message,BackType *back);
void prase_sense(char* buf, COMMessage *message,BackType *back);
void prase_initiate(char* buf, COMMessage *message,BackType *back);
void prase_sense_sweep(char* buf, COMMessage *message,BackType *back);
void prase_sense_detector(char *buf,COMMessage *message,BackType *back);
void prase_sense_demodulation(char* buf,COMMessage *message,BackType *back);
void prase_sense_waveout(char* buf,COMMessage *message,BackType *back);
void prase_sense_freq(char* buf, COMMessage *message,BackType *back);
void prase_sense_gps(char *buf, COMMessage *message,BackType *back);
void prase_sense_win(char *buf, COMMessage *message, BackType *back);
void prase_sense_gain(char* buf, COMMessage *message,BackType *back);
void prase_sense_tune(char* buf, COMMessage *message,BackType *back);
void prase_sense_dwell(char* buf, COMMessage *message,BackType *back);
void prase_sense_timezone(char* buf, COMMessage *message,BackType *back);
double prase_parameter(char* buf);
//void prase_data(char* buf);


long long int str2int(char *buf);
double str2double(char *buf);
#endif
