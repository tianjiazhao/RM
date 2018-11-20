#include "serial.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
unsigned char pitch_bit_;
unsigned char yaw_bit_;
int OpenPort(const char *Portname)
{
    int fd;
    fd = open(Portname,O_RDWR);
    if(-1 == fd)
       { printf("The port open error!");
        return 0;
        }

    else {
        fcntl(fd,F_SETFL,0);   //读取串口的信息
    }
    return fd;

}

int configurePort(int fd)
{
    struct termios port_set;

    cfsetispeed(&port_set,B115200);
    cfsetospeed(&port_set,B115200);
    //No parity
    port_set.c_cflag &= ~PARENB;         //无校验
    port_set.c_cflag &= ~CSTOPB;         //停止位:1bit
    port_set.c_cflag &= ~CSIZE;          //清除数据位掩码
    port_set.c_cflag |=  CS8;
//    port_set.c_cflag |=  CLOCAL;
//    tcgetattr(fd,&port_set);
    tcsetattr(fd,TCSANOW,&port_set);
    return (fd);

}
bool SendData(int fd,double *data)
{
    unsigned char send_bytes[]={0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xBB};
//    unsigned char send_bytes[]={1,1,1};
    if(NULL==data)
    {
        if(8==write(fd,send_bytes,8))
            return true;
        return false;
    }
//    short *data_prt=(short *) (send_bytes +1);
//    data_prt[0]=(short)data[0];
//    data_prt[1]=(short)data[1];
//    data_prt[2]=(short)data[2];
    if(data[0]<0)
     {   pitch_bit_ = 0x00;
         data[0]=-data[0];
    }
    else pitch_bit_ = 0x01;

    if(data[1]<0)
     {   yaw_bit_ = 0x00;
         data[1]=-data[1];
    }
    else yaw_bit_ = 0x01;

    send_bytes[1] = data[0];
    send_bytes[2] = pitch_bit_;
    send_bytes[3] = data[1];
    send_bytes[4] = yaw_bit_;
    send_bytes[5] = data[2];
    if(8==write(fd,send_bytes,8))
    {

        return true;
    }
    return false;
}
