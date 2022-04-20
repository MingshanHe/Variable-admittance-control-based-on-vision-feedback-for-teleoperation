#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

class TrackPoint
{
public:
    TrackPoint(ros::NodeHandle nh_, double frequency);
    ~TrackPoint(){};
public:
    void run();
    bool get_rotation_matrix(std::string from_frame, std::string to_frame);
    void publish_track_node();
private:
    void IMU1_Sub_Callback(const geometry_msgs::Vector3 &msg);
    void IMU2_Sub_Callback(const geometry_msgs::Vector3 &msg);

private:
    ros::NodeHandle             nh;
    ros::Subscriber             IMU1_Sub;
    ros::Subscriber             IMU2_Sub;
    ros::Publisher              Track_Pub;
    ros::Rate                   loop_rate;
    tf::Transform               transform;
    tf::StampedTransform        listen_transform;
    tf::Quaternion              quaternion;
    tf::Vector3                 track_point;
private:
    double                      upperArm;
    double                      foreArm;
};