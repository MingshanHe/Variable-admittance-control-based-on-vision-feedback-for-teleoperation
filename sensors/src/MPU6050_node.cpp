#include "MPU6050/MPU6050.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosservice");

    ros::NodeHandle nh;
    std::string usb;
    std::string imu;
    double frequency;
    if (!nh.getParam("usb", usb)) { ROS_ERROR("Couldn't retrieve the usb."); return -1;}
    if (!nh.getParam("imu", imu)) { ROS_ERROR("Couldn't retrieve the imu."); return -1;}
    if (!nh.getParam("frequency", frequency)) { ROS_ERROR("Couldn't retrieve the frequency."); return -1;}

    MPU6050 mpu6050 = MPU6050 (usb.c_str(), B115200, imu.c_str(), nh, frequency);
}