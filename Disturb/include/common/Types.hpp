#ifndef TYPES_H
#define TYPES_H

#include <stdlib.h>
#include <string>


typedef unsigned char BYTE;
typedef unsigned short Uint2;
typedef unsigned int	Uint4;
typedef unsigned long long int Uint8;
typedef unsigned long long int Freq64;


enum Modulation
{
	M_CW=0,
	M_FM=1,
	M_AM=2,
	M_GAUSS=3,
	M_FSK=4,
	M_ASK=5,
	M_MSK=6,
	M_QPSK=7,
	M_8PSK=8,
	M_16QAM=9
};


typedef struct {
	double x;
	double y;
	double z;
} gpos;

typedef struct {
	int dtime;
	float papr;
	float plocal;
	float pmaster;
}TDOA_Data;


void writebuf(void** wp,void *data,int nbits);
void readbuf(void** wp,void *data,int nbits);
std::string ModtoStr(Modulation mod);

#endif

