#include "TrackPoint/trackpoint.h"
TrackPoint::TrackPoint(ros::NodeHandle nh_):
    nh(nh_)
{
    nh = nh_;
    IMU1_Sub = nh.subscribe("/IMU1/IMU", 2, &TrackPoint::IMU1_Sub_Callback, this);
    IMU2_Sub = nh.subscribe("/IMU2/IMU", 2, &TrackPoint::IMU2_Sub_Callback, this);
    Track_Pub = nh.advertise<geometry_msgs::Vector3>("/track_node/trackpoint", 1);
}

TrackPoint::~TrackPoint()
{
}

void TrackPoint::IMU1_Sub_Callback(const geometry_msgs::Vector3 &msg)
{
    // double a = 1;
    transform.setOrigin(tf::Vector3(0.0, 0.0, upperArm));
    quaternion.setRPY(msg.x, msg.y, msg.z);
    transform.setRotation(quaternion);
    IMU1_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "upperArm"));
}

void TrackPoint::IMU2_Sub_Callback(const geometry_msgs::Vector3 &msg)
{
    // double b = 1;
    transform.setOrigin(tf::Vector3(0.0, 0.0, foreArm));
    quaternion.setRPY(msg.x, msg.y, msg.z);
    transform.setRotation(quaternion);
    IMU1_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "upperArm", "foreArm"));
}

// bool TrackPoint::get_rotation_matrix(std::string from_frame, std::string to_frame) {
//     // tf::StampedTransform transform;

//         // IMU_TF_listener.lookupTransform(from_frame, to_frame, ros::Time(0), transform);
//         // track_point = transform.getOrigin();
//         return true;
// }

void TrackPoint::publish_track_node(){

    // if(get_rotation_matrix("world", "foreArm")){
        geometry_msgs::Vector3 msg;

        msg.x = track_point.x();
        msg.y = track_point.y();
        msg.z = track_point.z();

        Track_Pub.publish(msg);
    // }
}

void TrackPoint::run()
{
    while (ros::ok())
    {
        ros::spinOnce();

        // loop_rate.sleep();

        publish_track_node();
    }
}