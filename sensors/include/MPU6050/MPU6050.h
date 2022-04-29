#ifndef MPU6050_H
#define MPU6050_H
#include "MPU6050/Serial.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"

class MPU6050 : public Serial
{
public:
    MPU6050(const char * Device_, int Baudrate_, const char * Name_, ros::NodeHandle nh_);
    ~MPU6050(){};
public:
    void Read_Data();
    void    run();

public:
    float                   angle_x;
    float                   angle_y;
    float                   angle_z;

    float                   Ang[3];
    float                   Vel[3];
    float                   Acc[3];

    int                     num;
private:
    int                     Serial_nFd;
    unsigned char           Serial_k;
    int                     Serial_rxflag;
    int                     Serial_len;

    unsigned char           leftknee_recdata[1];
    unsigned char           Re_buf[11];

    const char *            Name;

    ros::NodeHandle         nh;
    ros::Timer              timer_;
    ros::Publisher          IMU_Pub;
    // ros::Rate           loop_rate;

    geometry_msgs::Vector3  msg;
};
#endif