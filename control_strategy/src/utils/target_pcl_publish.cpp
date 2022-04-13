#include "utils/target_pcl_publish.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("/target_pcl", 1);

    PointCloud::Ptr msg (new PointCloud);
    msg->header.frame_id = "world";
    msg->height = 2;
    msg->width = 1;
    msg->points.push_back (pcl::PointXYZ(0.7, 0.0, 0.1));
    msg->points.push_back (pcl::PointXYZ(-0.7, 0.0, 0.1));

    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish (msg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}