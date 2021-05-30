#ifndef INVERSE_KINEMETICS_H_
#define INVERSE_KINEMETICS_H_

#include <iostream>
#include <string>
#include <vector>
#include <exception>

#include <ros/ros.h>
#include <tf/tf.h>

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
class InverseKinematicSolver
{
public:
	InverseKinematicSolver(const ros::NodeHandle& nh);

	~InverseKinematicSolver()
	{}

	bool calcIK(const geometry_msgs::PoseStamped &request, 
							std::vector<double> &result);

private:
	ros::NodeHandle nh_;
	std::string robot_desc_string_;
	std::vector<geometry_msgs::PoseStamped> result_;

	boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
	boost::shared_ptr<KDL::ChainIkSolverPos_NR> ik_solver_;
	boost::shared_ptr<KDL::ChainIkSolverVel_wdls> ik_solver_vel_;

	KDL::Chain chain_;
	KDL::Frame req_;
};

} // namespace kinematics_solver

#endif