
#include     <stdio.h>      /*±ê×ŒÊäÈëÊä³ö¶šÒå*/
#include     <stdlib.h>     /*±ê×Œº¯Êý¿â¶šÒå*/
#include     <unistd.h>     /*Unix±ê×Œº¯Êý¶šÒå*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>   /**/
#include     <fcntl.h>      /*ÎÄŒþ¿ØÖÆ¶šÒå*/
#include     <termios.h>    /*PPSIXÖÕ¶Ë¿ØÖÆ¶šÒå*/
#include     <errno.h>      /*ŽíÎóºÅ¶šÒå*/
#include     <string.h>
#include     <iostream>
#include     <common/GPS.h>
using namespace std;


int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
	    B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300,
	    38400,  19200,  9600, 4800, 2400, 1200,  300, };
void set_speed(int fd, int speed)
{
  int   i;
  int   status;
  struct termios   Opt;
  tcgetattr(fd, &Opt);
  for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
   {
   	if  (speed == name_arr[i])
   	{
   	    tcflush(fd, TCIOFLUSH);
    	cfsetispeed(&Opt, speed_arr[i]);
    	cfsetospeed(&Opt, speed_arr[i]);
    	status = tcsetattr(fd, TCSANOW, &Opt);
    	if  (status != 0)
            perror("tcsetattr fd1");
     	return;
     	}
   tcflush(fd,TCIOFLUSH);
   }
}

int set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
 if  ( tcgetattr( fd,&options)  !=  0)
  {
  	perror("SetupSerial 1");
  	return(false);
  }
  options.c_cflag &= ~CSIZE;
  switch (databits) /*ÉèÖÃÊýŸÝÎ»Êý*/
  {
  	case 7:
  		options.c_cflag |= CS7;
  		break;
  	case 8:
		options.c_cflag |= CS8;
		break;
	default:
		fprintf(stderr,"Unsupported data size\n");
		return (false);
	}
  switch (parity)
  	{
  	case 'n':
	case 'N':
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
		options.c_iflag &= ~INPCK;     /* Enable parity checking */
		break;
	case 'o':
	case 'O':
		options.c_cflag |= (PARODD | PARENB);  /* ÉèÖÃÎªÆæÐ§Ñé*/ 
		options.c_iflag |= INPCK;             /* Disnable parity checking */
		break;
	case 'e':
	case 'E':
		options.c_cflag |= PARENB;     /* Enable parity */
		options.c_cflag &= ~PARODD;   /* ×ª»»ÎªÅŒÐ§Ñé*/  
		options.c_iflag |= INPCK;       /* Disnable parity checking */
		break;
	case 'S':
	case 's':  /*as no parity*/
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported parity\n");
		return (false);
		}
  /* ÉèÖÃÍ£Ö¹Î»*/   
  switch (stopbits)
  	{
  	case 1:
  		options.c_cflag &= ~CSTOPB;
		break;
	case 2:
		options.c_cflag |= CSTOPB;
		break;
	default:
		fprintf(stderr,"Unsupported stop bits\n");
		return (false);
	}
  /* Set input parity option */
  if (parity != 'n')
  		options.c_iflag |= INPCK;
    options.c_cc[VTIME] = 150; // 15 seconds
    options.c_cc[VMIN] = 0;

  tcflush(fd,TCIFLUSH); /* Update the options and do it NOW */
  if (tcsetattr(fd,TCSANOW,&options) != 0)
  	{
  		perror("SetupSerial 3");
		return (false);
	}
  return (true);
 }
/**
*@breif Žò¿ªŽ®¿Ú
*/
int OpenDev(char *Dev)
{
int	fd = open( Dev, O_RDWR );         //| O_NOCTTY | O_NDELAY
	if (-1 == fd)
		{ /*ÉèÖÃÊýŸÝÎ»Êý*/
			perror("Can't Open Serial Port");
			return -1;
		}
	else
	return fd;

}
/**
*@breif 	main()
*/

int gpsload(char *comname)
{
   int fd=-1;
   fd= OpenDev(comname);
   if (fd>0)
     set_speed(fd,38400);
   else{
	printf("Can't Open Serial Port!\n");
	fd=-1;
   }
   if (set_Parity(fd,8,1,'N')== false)
   {
	printf("Set Parity Error\n");
	fd=-1;
   }
   return fd;
}

int gstr2int(char *buf)
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



int gpsread(int fd, int buf[])
{
   char rbuf[512];
   char buff[1024];
   int nread=0;
   int sread=0;
   char command[6];
   command[5]='\0';
   char Nd[3],Nm[3],Ns[5],Ed[4],Em[3],Es[5];
   Nd[2]='\0';
   Nm[2]='\0';
   Ns[4]='\0';
   Ed[3]='\0';
   Em[2]='\0';
   Es[4]='\0';
   while(nread<512){
     sread=read(fd,rbuf,512);    
     memcpy(buff+nread,rbuf,sread);
     nread=nread+sread;       
   }
   //buff[nread]='\0';
   //cout<<"buf="<<buff<<endl;
   for(int i=0;i<nread;i++)
   {
      if((buff[i]=='$') and (nread>(i+43))) //43 is the length of GPRMC
      {   
         memcpy(command,buff+i+1,5);
         if (strcmp(command,"GPRMC")==0)
         {

           memcpy(Nd,buff+i+20,2);
           memcpy(Nm,buff+i+22,2);
           memcpy(Ns,buff+i+25,4);//,
           memcpy(Ed,buff+i+32,3);//,N,
           memcpy(Em,buff+i+35,2);//,
           memcpy(Es,buff+i+38,4);
           //cout<<"string=";
           //cout<<Nd<<" "<<Nm<<" "<<Ns<<" "<<Ed<<" "<<Em<<" "<<Es<<" "<<endl;
           buf[0]=gstr2int(Nd);
           buf[1]=gstr2int(Nm);
           buf[2]=gstr2int(Ns);
           buf[3]=gstr2int(Ed);
           buf[4]=gstr2int(Em);
           buf[5]=gstr2int(Es);
           buf[6]=(int) (buff[i+30]=='N');
           buf[7]=(int) (buff[i+43]=='E');
           return 8;
         }
      }
         
   }
  
   return 0;

}


