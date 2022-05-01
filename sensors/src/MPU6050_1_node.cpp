#include "MPU6050/MPU6050.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "rosservice");
    ros::NodeHandle nh;

    MPU6050 mpu6050 = MPU6050 ("/dev/ttyUSB0", B115200, nh);

    mpu6050.run();
}