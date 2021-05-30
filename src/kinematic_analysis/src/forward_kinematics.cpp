#include "kinematic_analysis/forward_kinematics.h"

namespace kinematics_solver
{
ForwardKinematicSolver::ForwardKinematicSolver(const ros::NodeHandle& nh) : 
		nh_(nh)
{
	if(!nh_.getParam("robot_description", robot_desc_string_))
	{
		throw std::runtime_error("Robot description parameter couldn't find in the parameter server!");
	}

	nh_.param<std::string>("/base_link", base_link_, "base_link");

	nh_.param<std::string>("/tip_link", tip_link_, "link_6");

	KDL::Tree tree;
	if(!kdl_parser::treeFromString(robot_desc_string_, tree))
	{
		throw std::runtime_error("Failed to construct KDL tree");
	}

	KDL::SegmentMap segments = tree.getSegments();

	for(auto segment = segments.begin(); segment != segments.end(); segment++)
	{
		std::cout << segment->first << std::endl;
	}

	const int num_joints = tree.getNrOfJoints();
	ROS_INFO("%d numbers of joints are used", num_joints);

	if(!tree.getChain(base_link_, tip_link_, chain_))
	{
		throw std::runtime_error("Could not get chain!");
	}

	fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
}

geometry_msgs::PoseStamped ForwardKinematicSolver::calcFK(const std::vector<double> &joint_arr)
{
	geometry_msgs::PoseStamped result;

	KDL::JntArray q(joint_arr.size());
	printf("joint_arr size:%zu\n", joint_arr.size());
	for(int i = 0; i < joint_arr.size(); i++)
	{
		q.data[i] = joint_arr[i];
	}

	KDL::Frame result_f;
	fk_solver_->JntToCart(q, result_f);

	double roll, pitch, yaw;
	result_f.M.GetRPY(roll, pitch, yaw);
	
	double qx, qy, qz, qw;
	result_f.M.GetQuaternion(qx, qy, qz, qw);
	
	std::cout << "Forward Kinematic solution: " << std::endl;
	std::cout << "Position: " << result_f.p.data[0] << " " << result_f.p.data[1]  << " " << result_f.p.data[2] << std::endl;
	std::cout << "Orientation: " << roll << " " << pitch  << " " << yaw << std::endl;
	std::cout << "Quaternion: " << qx << ", " << qy << ", " << qz << ", " << qw << std::endl;
	std::cout << "---------------------\n";

	result.pose.position.x = result_f.p.data[0];
	result.pose.position.y = result_f.p.data[1];
	result.pose.position.z = result_f.p.data[2];
	result.pose.orientation.x = qx;
	result.pose.orientation.y = qy;
	result.pose.orientation.z = qz;
	result.pose.orientation.w = qw;

	return result;
}

} // namespace kinematics_solver



int main(int argc, char** argv)
{
	ros::init(argc, argv, "forward_kinematics");

	ros::NodeHandle nh;

	kinematics_solver::ForwardKinematicSolver fk(nh);

	std::vector<double> joint_arr = {0.0, 1.57, 0.0, 0.0, 0.0, 0.0};
	std::cout << "Input angles: " << std::endl;
	for(auto j : joint_arr)
	{
		std::cout << j << std::endl;
	}
	fk.calcFK(joint_arr);
	return 0;
}