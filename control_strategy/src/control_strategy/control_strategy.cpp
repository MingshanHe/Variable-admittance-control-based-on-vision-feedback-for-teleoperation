#include "control_strategy/control_strategy.h"

Control_Strategy::Control_Strategy(
    const ros::NodeHandle   &nh_,
    double                  frequency,
    std::vector<double>     home_joint_position_,
    std::vector<double>     work_start_pose_,
    std::vector<double>     grasp_pose_,
    std::vector<double>     workspace_limits_):
    nh(nh_), loop_rate(frequency),
    home_joint_position(home_joint_position_.data()),
    work_start_pose(work_start_pose_.data()),
    grasp_pose(grasp_pose_.data()),
    workspace_limits(workspace_limits_)
{
    kinematics_base.init(nh);
    // ROS Pub&Sub
    Joint_Traj_Pub  = nh.advertise<trajectory_msgs::JointTrajectory>(Joint_Trajectory_Pub_Topic, 1, true);
    Joint_Traj_Sub  = nh.subscribe(Joint_Trajectory_Sub_Topic, 5, &Control_Strategy::Joint_State_Cb, this, ros::TransportHints().reliable().tcpNoDelay());
    Human_IMU_Sub   = nh.subscribe(Human_IMU_Sub_Topic,5, &Control_Strategy::Human_IMU_Cb, this, ros::TransportHints().reliable().tcpNoDelay());
    // traj intialize
    traj.joint_names.push_back("shoulder_pan_joint");
    traj.joint_names.push_back("shoulder_lift_joint");
    traj.joint_names.push_back("elbow_joint");
    traj.joint_names.push_back("wrist_1_joint");
    traj.joint_names.push_back("wrist_2_joint");
    traj.joint_names.push_back("wrist_3_joint");

    Jnt_Position.resize(6);
    Cart_Robot_Pose.p.Zero();
    Cart_Robot_Pose.M.Identity();
    Cart_Human_Pose.p.Zero();
    Cart_Human_Pose.M.Identity();
    ros::spinOnce();
    loop_rate.sleep();
}
//!-                    Robot Control Functions                      -!//
void Control_Strategy::EfficiencyMove(){
    Cart_Human_Pose.M = Rotation.Quaternion(work_start_pose[3],work_start_pose[4],work_start_pose[5],work_start_pose[6]);
    ik_pos_solver_->CartToJnt(Jnt_Position, Cart_Human_Pose, Jnt_Position_cmd);

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

void Control_Strategy::VariableCompliantMove(){
    //TODO: Add Variable Compliant Move
}
//!-                    Robot Initial Functions                      -!//
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
//!-                    Callbacks                       -!//
void Control_Strategy::Joint_State_Cb(const control_msgs::JointTrajectoryControllerState &msg){

    for (size_t i = 0; i < 6; i++)
    {
        Jnt_Position(i) = msg.actual.positions[i];
    }
}

void Control_Strategy::Human_IMU_Cb(const geometry_msgs::Vector3 &msg){
    Cart_Human_Pose.p[0] = msg.x;
    Cart_Human_Pose.p[1] = msg.y;
    Cart_Human_Pose.p[2] = msg.z;
}

//!-                    UTILIZATION                      -!//

bool Control_Strategy::get_rotation_matrix(Matrix6d & rotation_matrix,
    tf::TransformListener & listener,
    std::string from_frame,
    std::string to_frame) {
    tf::StampedTransform transform;
    Matrix3d rotation_from_to;
    try {
        listener.lookupTransform(from_frame, to_frame,
                                ros::Time(0), transform);
        tf::matrixTFToEigen(transform.getBasis(), rotation_from_to);
        rotation_matrix.setZero();
        rotation_matrix.topLeftCorner(3, 3) = rotation_from_to;
        rotation_matrix.bottomRightCorner(3, 3) = rotation_from_to;
    }
    catch (tf::TransformException ex) {
        rotation_matrix.setZero();
        ROS_WARN_STREAM_THROTTLE(1, "Waiting for TF from: " << from_frame << " to: " << to_frame );
        return false;
    }
    return true;
}

void Control_Strategy::Switch_Controller(const int &cognition)
{
    std::string cartesian_position_controller("cartesian_position_controller");
    std::string cartesian_velocity_controller("cartesian_velocity_controller");
    switch (cognition)
    {
    case 0:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_position_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_position_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_position_controller' Failed. Please Check Code");
        }
        break;
    case 1:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_velocity_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_velocity_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_velocity_controller' Failed. Please Check Code");
        }
        break;
    case 2:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_position_controller);
        stop_controllers.push_back(cartesian_velocity_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_position_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_position_controller' Failed. Please Check Code");
        }
        break;
    case 3:
        start_controllers.clear();
        stop_controllers.clear();

        start_controllers.push_back(cartesian_velocity_controller);
        stop_controllers.push_back(cartesian_position_controller);
        switch_controller_srv.request.start_controllers = start_controllers;
        switch_controller_srv.request.stop_controllers = stop_controllers;
        switch_controller_srv.request.strictness = 2;
        if(switch_controller_client.call(switch_controller_srv))
        {
            ROS_INFO("Switch 'cartesian_velocity_controller' Successfully.");
        }
        else
        {
            ROS_ERROR("Switch 'cartesian_velocity_controller' Failed. Please Check Code");
        }
        break;
    default:
        ROS_ERROR("Switch Controller Cognition Failed. Please Check Code and Choose ( 0 or 1 ).");
        break;
    }
}

bool Control_Strategy::DetectWorkspace(){
    this->fk_pos_solver_->JntToCart(Jnt_Position, Cart_Robot_Pose);
    //TODO: detect if the position fo cart pose in the desired workspace
    //TODO: if it is in return true else false
    double x = Cart_Robot_Pose.p.x();
    double y = Cart_Robot_Pose.p.y();
    double z = Cart_Robot_Pose.p.z();
    if (
        (((x>workspace_limits[0])and(x<workspace_limits[1]))and((y>workspace_limits[2])and(y<workspace_limits[3]))and((z>workspace_limits[4])and(z<workspace_limits[5])))and
        (((x>workspace_limits[7])and(x<workspace_limits[8]))and((y>workspace_limits[9])and(y<workspace_limits[10]))and((z>workspace_limits[11])and(z<workspace_limits[12])))
    )
    {
        //TODO: Variable Admittance Control
        VariableCompliantMove();
        return true;
    }
    else
    {
        //TODO: Position Control
        EfficiencyMove();
        return false;
    }
}
void Control_Strategy::run(){

    while (nh_.ok()) {
        ros::spinOnce();

        loop_rate.sleep();

        if(DetectWorkspace()){
            VariableCompliantMove();
        }
        else{
            EfficiencyMove();
        }
    }
}