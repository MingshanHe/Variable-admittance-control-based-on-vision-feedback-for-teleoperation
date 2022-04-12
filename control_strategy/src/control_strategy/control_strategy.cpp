// #include "control_strategy/control_strategy.h"
#include "control_strategy.h"

Control_Strategy::Control_Strategy(
    const ros::NodeHandle &nh_,
    double                  frequency,
    std::vector<double>     home_joint_position_,
    std::vector<double>     work_start_pose_,
    std::vector<double>     grasp_pose_):
    nh(nh_), loop_rate(frequency),
    home_joint_position(home_joint_position_.data()),
    work_start_pose(work_start_pose_.data()),
    grasp_pose(grasp_pose_.data())
{
    kinematics_base.init(nh);
    // ROS Pub&Sub
    Joint_Traj_Pub  = nh.advertise<trajectory_msgs::JointTrajectory>(Joint_Trajectory_Pub_Topic, 1, true);
    Joint_Traj_Sub  = nh.subscribe(Joint_Trajectory_Sub_Topic, 5, &Control_Strategy::Joint_State_Cb, this, ros::TransportHints().reliable().tcpNoDelay());
    // traj intialize
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    Jnt_Position.resize(6);
    ros::spinOnce();
    loop_rate.sleep();
}


void Control_Strategy::Go_Home(void)
{

    trajectory_msgs::JointTrajectoryPoint msg;
    for (size_t i = 0; i < 6; i++)
    {
        msg.positions.push_back(home_joint_position[i]);
        msg.velocities.push_back(0.2);
    }
    msg.time_from_start = ros::Duration(1);
    traj.points.clear();
    traj.points.push_back(msg);

    Joint_Traj_Pub.publish(traj);
    ros::spinOnce();
    loop_rate.sleep();
}

void Control_Strategy::Go_Work(void)
{
    T_Base_Goal.p(0) = work_start_pose[0];
    T_Base_Goal.p(1) = work_start_pose[1];
    T_Base_Goal.p(2) = work_start_pose[2];
    T_Base_Goal.M    = Rotation.Quaternion(work_start_pose[3],work_start_pose[4],work_start_pose[5],work_start_pose[6]);

    Jnt_Position_cmd = kinematics_base.inverse_kinematics(Jnt_Position, T_Base_Goal);

    trajectory_msgs::JointTrajectoryPoint msg;
    for (size_t i = 0; i < 6; i++)
    {
        msg.positions.push_back(Jnt_Position_cmd(i));
    }
    msg.time_from_start = ros::Duration(1);
    traj.points.clear();
    traj.points.push_back(msg);

    Joint_Traj_Pub.publish(traj);
    ros::spinOnce();
    loop_rate.sleep();
}

void Control_Strategy::Go(Eigen::Vector3d Position)
{
    ros::Rate loop_rate(10);
    trajectory_msgs::JointTrajectory msgs;
    msgs.joint_names.push_back("shoulder_pan_joint");
    msgs.joint_names.push_back("shoulder_lift_joint");
    msgs.joint_names.push_back("elbow_joint");
    msgs.joint_names.push_back("wrist_1_joint");
    msgs.joint_names.push_back("wrist_2_joint");
    msgs.joint_names.push_back("wrist_3_joint");

    trajectory_msgs::JointTrajectoryPoint msg;
    msg.positions.push_back(0.0);
    msg.positions.push_back(-2.33);
    msg.positions.push_back(1.57);
    msg.positions.push_back(0.0);
    msg.positions.push_back(0.0);
    msg.positions.push_back(0.0);
    msg.time_from_start = ros::Duration(1);
    msgs.points.push_back(msg);

    size_t i = 3;
    while (i>0)
    {
        Joint_Traj_Pub.publish(msgs);
        ros::spinOnce();
        loop_rate.sleep();
        i--;
    }
}

void Control_Strategy::Joint_State_Cb(const control_msgs::JointTrajectoryControllerState &msg)
{
    for (size_t i = 0; i < 6; i++)
    {
        Jnt_Position(i) = msg.actual.positions[i];
    }
}