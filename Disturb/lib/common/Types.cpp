#include <stdlib.h>
#include <string.h>
#include <iostream>



#include <common/Types.hpp>
using namespace std;
void writebuf(void** wp,void *data,int nbits)
{

   memcpy(*wp,data,nbits);  
   *wp=(void *)((BYTE *)*wp+nbits);
}


void readbuf(void** wp,void *data,int nbits)
{
   memcpy(data,*wp,nbits);
   *wp=(void *)((BYTE *)*wp+nbits);
}

std::string ModtoStr(Modulation mod)
{
	switch(mod){
		case M_CW: return string("CW");
		case M_FM: return string("FM");
		case M_AM: return string("AM");
		case M_GAUSS: return string("GAUSS");
		case M_FSK: return string("FSK");
		case M_ASK: return string("ASK");
		case M_MSK: return string("MSK");
		case M_QPSK: return string("QPSK");
		case M_8PSK: return string("8PSK");
		case M_16QAM: return string("16QAM");
		default:return string("UnknownModulation");
	}
}


