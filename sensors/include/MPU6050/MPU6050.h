#ifndef MPU6050_H
#define MPU6050_H
#include "MPU6050/Serial.h"
#include "ros/ros.h"

class MPU6050 : public Serial
{
public:
    MPU6050(char * Device_, int Baudrate_, char * Name_, ros::NodeHandle nh, double frequency);
    ~MPU6050(){};
public:
    float * Read_Data();
private:
    void    Callback(const ros::TimerEvent& event);

public:
    float angle_x;
    float angle_y;
    float angle_z;

    float           Ang[3];
    float           Vel[3];
    float           Acc[3];

    int             num;
private:
    int                 Serial_nFd;
    unsigned char       Serial_k;
    int                 Serial_rxflag;
    int                 Serial_len;

    unsigned char       leftknee_recdata[1];
    unsigned char       Re_buf[11];

    char *              Name;

    ros::NodeHandle     nh;
    ros::Timer          timer_;
};
#endif