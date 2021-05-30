#ifndef FORWARD_KINEMETICS_H_
#define FORWARD_KINEMETICS_H_

#include <iostream>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>

namespace kinematics_solver
{
class ForwardKinematicSolver
{
public:
	ForwardKinematicSolver(const ros::NodeHandle& nh);

	~ForwardKinematicSolver()
	{}

	geometry_msgs::PoseStamped calcFK(const std::vector<double> &joint_arr);

private:
	ros::NodeHandle nh_;
	std::string robot_desc_string_;
	std::vector<geometry_msgs::PoseStamped> result_;

	std::string base_link_;
	std::string tip_link_;

	KDL::Chain chain_;
	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
};

} // namespace kinematics_solver

#endif