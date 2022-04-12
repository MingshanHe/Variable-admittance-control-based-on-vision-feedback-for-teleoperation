#ifndef CONTROL_STRATEGY_H
#define CONTROL_STRATEGY_H
#include "ros/ros.h"
#include <iostream>
#include <string>

#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "control_msgs/JointTrajectoryControllerState.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "control_msgs/GripperCommand.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

#include "kinematics_base.h"

#define Joint_Trajectory_Pub_Topic          "/pos_joint_traj_controller/command"
#define Joint_Trajectory_Sub_Topic          "/pos_joint_traj_controller/state"
using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class Control_Strategy: Kinematics_Base
{
private:
    /* data */
public:
    Control_Strategy(const ros::NodeHandle &nh_,
                        double              frequency,
                        std::vector<double> home_joint_position_,
                        std::vector<double> work_start_pose_,
                        std::vector<double> grasp_pose_);
    ~Control_Strategy(){};
public:
    void Go_Home(void);
    void Go_Work(void);
    void Go(Eigen::Vector3d Position);
public:
    void Joint_State_Cb(const control_msgs::JointTrajectoryControllerState &msg);
public:
    KDL::JntArray                                           Jnt_Position;
    KDL::JntArray                                           Jnt_Position_cmd;
    KDL::Rotation                                           Rotation;
    KDL::Frame                                              T_Base_Goal;
private:
    ros::NodeHandle                                         nh;
    Kinematics_Base                                         kinematics_base;
    ros::Publisher                                          Joint_Traj_Pub;
    ros::Subscriber                                         Joint_Traj_Sub;
    ros::Rate                                               loop_rate;

private:
    trajectory_msgs::JointTrajectory                        traj;
    Vector6d                                                home_joint_position;
    Vector7d                                                work_start_pose;
    Vector7d                                                grasp_pose;

};



#endif