#include "MPU6050/MPU6050.h"

MPU6050::MPU6050(const char * Device_, int Baudrate_, ros::NodeHandle nh_) :
    Serial()
{
    nh = nh_;
    std::string name_space = nh_.getNamespace();
    IMU_Pub = nh.advertise<geometry_msgs::Vector3>(name_space+"/IMU",2);

    Serial_nFd = this->Init(Device_,Baudrate_);

    Serial_k = 0;
    Serial_rxflag = 1;
    Serial_len=0;
}

void MPU6050::run(){
    ros::Rate loop_rate(115200);
    while (ros::ok())
    {
        Read_Data();
        msg.x = Ang[0]*PI/360;
        msg.y = Ang[1]*PI/360;
        msg.z = Ang[2]*PI/360;
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

            std::cout<<Ang[0]<<","<<Ang[1]<<","<<Ang[2]<<std::endl;
        }
    }
}

