#ifndef SERIAL_H
#define SERIAL_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <iostream>
#include <pthread.h>


class Serial
{
private:
    struct termios  Serial_stNew;
    struct termios  Serial_stOld;

    char *          Device;
    int             Baudrate;

    int             Serial_nFd;
public:
    Serial(){
        Serial_nFd = 0;
    };
    ~Serial(){};
public:
    int Init(char * Device_, int Baudrate_);
};

int Serial::Init(char * Device_, int Baudrate_)
{
    Device = Device_;
    Baudrate = Baudrate_;
    Serial_nFd = open(Device, O_RDWR|O_NOCTTY|O_NDELAY);   //打开串口USB0
    if(-1 == Serial_nFd)
    {
        perror("Open Serial Port Error!\n");
        return -1;
    }
    if( (fcntl(Serial_nFd, F_SETFL, 0)) < 0 )
    {
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }
    if(tcgetattr(Serial_nFd, &Serial_stOld) != 0)
    {
        perror("tcgetattr error!\n");
        return -1;
    }

    Serial_stNew = Serial_stOld;
    cfmakeraw(&Serial_stOld);//将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    /**1. tcgetattr函数用于获取与终端相关的参数。
    *参数fd为终端的文件描述符，返回的结果保存在termios结构体中
    */
    //set speed
    cfsetispeed(&Serial_stOld, Baudrate);   //设置波特率115200s
    //set databits
    Serial_stOld.c_cflag |= (CLOCAL|CREAD); //设置控制模式状态，本地连接，接收使能
    Serial_stOld.c_cflag &= ~CSIZE;         //字符长度，设置数据位之前一定要屏掉这个位
    Serial_stOld.c_cflag |= CS8;            //8位数据长度
    //set parity
    Serial_stOld.c_cflag &= ~PARENB;
    Serial_stOld.c_iflag &= ~INPCK;         //无奇偶检验位
    //set stopbits
    Serial_stOld.c_cflag &= ~CSTOPB;        //1位停止位

    //stNew.c_oflag = 0; //输出模式
    //stNew.c_lflag = 0; //不激活终端模式

    Serial_stOld.c_cc[VTIME]=0;             //指定所要读取字符的最小数量
    Serial_stOld.c_cc[VMIN]=1;              //指定读取第一个字符的等待时间，时间的单位为n*100ms
    //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    /**3. 设置新属性，TCSANOW：所有改变立即生效*/
    tcflush(Serial_nFd,TCIFLUSH);           //清空终端未完毕的输入/输出请求及数据。

    if( tcsetattr(Serial_nFd,TCSANOW,&Serial_stOld) != 0 )
    {
        perror("tcsetattr Error!\n");
        return -1;
    }
    return Serial_nFd;
}
#endif