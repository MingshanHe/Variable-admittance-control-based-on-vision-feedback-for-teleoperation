#include "MPU6050/MPU6050.h"

MPU6050::MPU6050(const char * Device_, int Baudrate_, const char * Name_, ros::NodeHandle nh_) :
    Serial()
{
    nh = nh_;
    IMU_Pub = nh.advertise<geometry_msgs::Vector3>("IMU",2);
    // Serial serial;
    // Serial_nFd = serial.Init(Device_, Baudrate_);
    Serial_nFd = this->Init(Device_,Baudrate_);

    Serial_k = 0;
    Serial_rxflag = 1;
    Serial_len=0;

    Name = Name_;
    // loop_rate = ros::Rate loop_rate_(125);
}

void MPU6050::run(){
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        float * Angle;

        Read_Data();
        // msg.x = 0.0;
        // msg.y = 1;
        // msg.z = 2;
        msg.x = Ang[0];
        msg.y = Ang[1];
        msg.z = Ang[2];
        IMU_Pub.publish(msg);
        loop_rate.sleep();
    }

}

void MPU6050::Read_Data()
{
    Serial_len = read(Serial_nFd, leftknee_recdata, 1);
    Re_buf[Serial_k] = leftknee_recdata[0];

    if(Serial_k == 0 && Re_buf[0] !=0x55)
    {
        Serial_rxflag = 0;
        Serial_k = 0;
    }
    if(Serial_rxflag)
    {
        Serial_k++;
        if(Serial_k == 11)
        {
            Serial_k = 0;
            if(Re_buf[0] == 0x55)
            {
                num = 0;
                switch (Re_buf[1])
                {
                case 0x51:
                    Acc[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*16;
                    Acc[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*16;
                    Acc[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*16;
                    break;
                case 0x52:
                    Vel[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*2000;
                    Vel[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*2000;
                    Vel[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*2000;
                    break;
                case 0x53:
                    Ang[0] = ((short)(Re_buf[3]<<8|Re_buf[2]))/32768.0*180;
                    Ang[1] = ((short)(Re_buf[5]<<8|Re_buf[4]))/32768.0*180;
                    Ang[2] = ((short)(Re_buf[7]<<8|Re_buf[6]))/32768.0*180;
                    break;
                }
            }
            // float Ang_Vel_Acc[3];
            // Ang_Vel_Acc[0] = Ang[0];
            // Ang_Vel_Acc[1] = Ang[1];
            // Ang_Vel_Acc[2] = Ang[2];
            // std::cout<<Ang[0]<<","<<Vel[0]<<","<<Acc[0]<<std::endl;
        }
    }
}

