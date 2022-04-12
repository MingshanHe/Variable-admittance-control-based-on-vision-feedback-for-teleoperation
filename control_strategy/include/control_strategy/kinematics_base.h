/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:34 
 * @Last Modified by:   MingshanHe 
 * @Last Modified time: 2021-12-05 04:09:34 
 * @Licence: MIT Licence
 */
#ifndef CARTESIAN_POSITION_CONTROLLER_KINEMATICS_BASE_H
#define CARTESIAN_POSITION_CONTROLLER_KINEMATICS_BASE_H

#include <urdf/model.h>

#include <ros/node_handle.h>
#include <ros/ros.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <vector>


class Kinematics_Base
{
public:
  Kinematics_Base() {}
  ~Kinematics_Base() {}

  bool init(ros::NodeHandle &n);

  KDL::JntArray inverse_kinematics(KDL::JntArray Jnt_Position_, KDL::Frame T_base_goal);

protected:
  ros::NodeHandle   nh_;
  KDL::Chain        kdl_chain_;
  KDL::JntArrayVel  joint_state_;
  KDL::JntArray     joint_effort_;

  struct limits_
  {
    KDL::JntArray min;
    KDL::JntArray max;
    KDL::JntArray center;
  } joint_limits_;

  KDL::JntArray                   Joint_Position;

  KDL::Rotation                   End_Pos_Rotation;
  KDL::Vector                     End_Pos_Vector;
  KDL::JntArray                   Jnt_Vel_Cmd_;      // Desired joint velocity
  KDL::Twist                      End_Vel_Cmd_;      // Desired end-effector velocity
  KDL::JntArray                   Jnt_Pos_Cmd_;      // Desired joint position
  KDL::Frame                      End_Pos_Cmd_;      // Desired end-effector position
  KDL::FrameVel                   End_Vel_;
  KDL::Frame                      End_Pos_;

  boost::shared_ptr<KDL::ChainFkSolverVel> fk_vel_solver_;
  boost::shared_ptr<KDL::ChainFkSolverPos> fk_pos_solver_;
  boost::shared_ptr<KDL::ChainIkSolverVel> ik_vel_solver_;
  boost::shared_ptr<KDL::ChainIkSolverPos> ik_pos_solver_;
};



#endif // CARTESIAN_VELOCITY_CONTROLLER_KINEMATICS_BASE_H
