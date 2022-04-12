#include "control_strategy/kinematics_base.h"

bool Kinematics_Base::init(ros::NodeHandle &n)
{
    nh_ = n;

    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    std::string name_space = nh_.getNamespace();
    std::cout<< "--------------------> name_space:  " << name_space << std::endl;

    if (!ros::param::search(name_space,"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("Kinematics_Base: No robot description (URDF)"
                        "found on parameter server (" << nh_.getNamespace() <<
                        "/robot_description)");
        return false;
    }

    if (!nh_.getParam( name_space + "/root_name", root_name))
    {
        ROS_ERROR_STREAM("Kinematics_Base: No root name found on "
                        "parameter server ("<<nh_.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_.getParam(name_space + "/tip_name", tip_name))
    {
        ROS_ERROR_STREAM("Kinematics_Base: No tip name found on "
                        "parameter server ("<<nh_.getNamespace()<<"/tip_name)");
        return false;
    }

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (nh_.hasParam(robot_description))
        nh_.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...",
                robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",
                robot_description.c_str());
        nh_.shutdown();
        return false;
    }

    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh_.shutdown();
        return false;
    }

    KDL::Tree kdl_tree;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree))
    {
        ROS_ERROR("Failed to construct kdl tree");
        nh_.shutdown();
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
        ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }


    // Parsing joint limits from urdf model along kdl chain
    std::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    std::shared_ptr<const urdf::Joint> joint_;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    int index;

    for (std::size_t i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);
        index = kdl_chain_.getNrOfJoints() - i - 1;

        if(joint_->limits){
        joint_limits_.min(index) = joint_->limits->lower;
        joint_limits_.max(index) = joint_->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) +
                                        joint_limits_.max(index))/2;
        }else{
        joint_limits_.min(index) = 0;
        joint_limits_.max(index) = 0;
        joint_limits_.center(index) = 0;
        }

        link_ = model.getLink(link_->getParent()->name);
    }


    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));
    fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    Jnt_Pos_Cmd_.resize(6);
    return true;
}

KDL::JntArray Kinematics_Base::inverse_kinematics(KDL::JntArray Jnt_Position_, KDL::Frame T_base_goal)
{
    // Joint_Position = Jnt_Position_;
    // fk_pos_solver_->JntToCart(Joint_Position, End_Pos_Cmd_);
    ik_pos_solver_->CartToJnt(Jnt_Position_, T_base_goal, Jnt_Pos_Cmd_);
    return Jnt_Pos_Cmd_;
}