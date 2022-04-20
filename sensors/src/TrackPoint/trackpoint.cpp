#include "TrackPoint/trackpoint.h"
TrackPoint::TrackPoint(ros::NodeHandle nh_, double frequency):
    nh(nh_), loop_rate(frequency)
{
    nh = nh_;
    IMU1_Sub = nh.subscribe("/IMU1/IMU", 2, &TrackPoint::IMU1_Sub_Callback, this);
    IMU2_Sub = nh.subscribe("/IMU2/IMU", 2, &TrackPoint::IMU2_Sub_Callback, this);
    Track_Pub = nh.advertise<geometry_msgs::Vector3>("/track_node/trackpoint", 1);
}

void TrackPoint::IMU1_Sub_Callback(const geometry_msgs::Vector3 &msg)
{
    tf::TransformBroadcaster    IMU1_TF_br;
    transform.setOrigin(tf::Vector3(0.0, 0.0, upperArm));
    quaternion.setRPY(msg.x, msg.y, msg.z);
    transform.setRotation(quaternion);
    IMU1_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "upperArm"));
}

void TrackPoint::IMU2_Sub_Callback(const geometry_msgs::Vector3 &msg)
{
    tf::TransformBroadcaster    IMU2_TF_br;
    transform.setOrigin(tf::Vector3(0.0, 0.0, foreArm));
    quaternion.setRPY(msg.x, msg.y, msg.z);
    transform.setRotation(quaternion);
    IMU2_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "upperArm", "foreArm"));
}

bool TrackPoint::get_rotation_matrix(std::string from_frame, std::string to_frame) {
    tf::TransformListener       IMU_TF_listener;
    try{
        IMU_TF_listener.lookupTransform(from_frame, to_frame, ros::Time(0), listen_transform);
        track_point = listen_transform.getOrigin();
        return true;
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return false;
    }
}

void TrackPoint::publish_track_node(){

    if(get_rotation_matrix("world", "foreArm")){
        geometry_msgs::Vector3 msg;

        msg.x = track_point.x();
        msg.y = track_point.y();
        msg.z = track_point.z();

        Track_Pub.publish(msg);
    }
}

void TrackPoint::run()
{
    while (ros::ok())
    {
        ros::spinOnce();

        loop_rate.sleep();

        publish_track_node();
    }
}