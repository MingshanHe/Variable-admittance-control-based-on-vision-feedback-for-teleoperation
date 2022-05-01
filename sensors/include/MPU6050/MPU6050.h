#include "MPU6050/Serial.h"
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#define PI 3.1415926

class MPU6050 : public Serial
{
public:
    MPU6050(const char * Device_, int Baudrate_, ros::NodeHandle nh_);
    ~MPU6050(){};
public:
    void Read_Data();
    void    run();

public:
    bool                    InitFlag;
    float                   InitAng[3];

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

    geometry_msgs::Vector3  msg;
};