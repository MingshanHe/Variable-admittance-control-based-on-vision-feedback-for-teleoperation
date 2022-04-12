#include "control_strategy/control_strategy.h"
#include "control_strategy/control_strategy_params.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rosservice");

    ros::NodeHandle nh;

    if (!nh.getParam("workspace_limits", workspace_limits)) { ROS_ERROR("Couldn't retrieve the workspace_limits."); return -1;}
    if (!nh.getParam("home_joint_position", home_joint_position)) { ROS_ERROR("Couldn't retrieve the home pose."); return -1;}
    if (!nh.getParam("work_start_pose", work_start_pose)) { ROS_ERROR("Couldn't retrieve the work start pose."); return -1;}
    if (!nh.getParam("grasp_pose", grasp_pose)) { ROS_ERROR("Couldn't retrieve the grasp pose."); return -1;}
    if (!nh.getParam("predict_map_size", predict_map_size)) { ROS_ERROR("Couldn't retrieve the predict map size."); return -1;}
    double frequency = 125;

    Control_Strategy control_strategy(nh, frequency, home_joint_position, work_start_pose, grasp_pose);

    ros::Rate loop_rate(frequency);
    sleep(2);
    control_strategy.Go_Home();
    std::cout<<"[STATE INFO] Go Home Successfully."<<std::endl;
    sleep(2);
    control_strategy.Go_Work();
    std::cout<<"[STATE INFO] Go Work Successfully."<<std::endl;
    sleep(2);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}