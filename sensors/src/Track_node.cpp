#include "TrackPoint/trackpoint.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "track_node");

    ros::NodeHandle nh;
    double frequency;

    if (!nh.getParam("frequency", frequency)) { ROS_ERROR("Couldn't retrieve the frequency."); return -1;}

    TrackPoint track = TrackPoint(nh, frequency);

    track.run();
}