#ifndef SERIAL_H
#define SERIAL_H
extern unsigned char pitch_bit_;
extern unsigned char yaw_bit_;
int OpenPort(const char *Portname);
int configurePort(int fd);
bool SendData(int fd,double *data);
#endif // SERIAL_H
