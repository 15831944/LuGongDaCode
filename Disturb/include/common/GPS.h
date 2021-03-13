#ifndef GPS_H
#define GPS_H


int gpsload(char *comname);
int gpsread(int fd,int *buf);
#endif
