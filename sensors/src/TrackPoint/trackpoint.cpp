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
    static tf::TransformBroadcaster    IMU1_TF_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(msg.x, msg.y, msg.z);
    x = msg.x;
    y = msg.y;
    z = msg.z;

    transform.setRotation(q);

    IMU1_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "shoulder"));

    static tf::TransformBroadcaster    IMU_TF_br;
    transform.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
    transform.setRotation(q);

    IMU_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "shoulder", "upperArm"));
}

void TrackPoint::IMU2_Sub_Callback(const geometry_msgs::Vector3 &msg)
{
    // static tf::TransformBroadcaster    IMU2_TF_br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
    // tf::Quaternion q;
    // q.setRPY(msg.x-x, msg.y-y, msg.z-z);
    // transform.setRotation(q);

    // IMU2_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "shoulder", "upperArm"));

    // static tf::TransformBroadcaster    TF_br;
    // transform.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
    // q.setRPY(msg.x-x, msg.y-y, msg.z-z);
    // transform.setRotation(q);

    // TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "upperArm", "foreArm"));
    static tf::TransformBroadcaster    IMU1_TF_br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    tf::Quaternion q;
    q.setRPY(msg.x, msg.y, msg.z);
    x = msg.x;
    y = msg.y;
    z = msg.z;

    transform.setRotation(q);

    IMU1_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "shoulder2"));

    static tf::TransformBroadcaster    IMU_TF_br;
    transform.setOrigin(tf::Vector3(0.0, 1.0, 0.0));
    transform.setRotation(q);

    IMU_TF_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "shoulder2", "upperArm2"));
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

    if(get_rotation_matrix("world", "upperArm")){
        geometry_msgs::Vector3 msg;

        msg.x = track_point.x();
        msg.y = track_point.y();
        msg.z = track_point.z();

        Track_Pub.publish(msg);
    }
}

void TrackPoint::run()
{
    // ros::Rate loop_rate_(9);
    // while (ros::ok())
    // {
    //     ros::spinOnce();

    //     loop_rate_.sleep();

    //     publish_track_node();
    // }
    ros::spin();
}